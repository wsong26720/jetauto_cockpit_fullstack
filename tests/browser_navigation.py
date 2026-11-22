"""Navigation UI contract, with every robot request intercepted locally."""
import asyncio,functools,http.server,json,os,threading,math
from pathlib import Path
from urllib.parse import urlparse
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1]

async def main():
    auto_test=os.environ.get('COCKPIT_BROWSER_AUTO_TEST')=='1'
    costmap_test=os.environ.get('COCKPIT_BROWSER_COSTMAP_TEST')=='1';reject_next=costmap_test
    server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(http.server.SimpleHTTPRequestHandler,directory=str(ROOT/'frontend')))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    posts=[];commands=[];errors=[];tasks=[]
    n={'status_code':0,'status_text':'IDLE','paused':False,'waypoints':[],
       'readiness':{'ready':False,'reasons':['请确认初始位姿'],'lifecycle':{'amcl':3}}}
    if auto_test:n['localization']={'state':'ROTATING','message':'正在低速原地旋转采集','rotation_degrees':90,'match':None}
    health={'control':{},'capabilities':{'mapping_view':True,'navigation':True},'nav_state':n,
            'control_ready':True,'estop_engaged':False,'hardware':{},'launches':[]}
    grid={'frame_id':'map','width':20,'height':20,'resolution':.1,'data':[0]*400,
          'origin':{'position':{'x':-1,'y':-1,'z':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}
    packet={'type':'mapping','available':True,'frame_id':'map','map_revision':1,'map_age_sec':.1,'map':grid,
            'pose':{'x':0,'y':0,'yaw':0,'age_sec':.01},'scan':None,
            'tf':{'robot':True,'scan':True,'issues':[]},'plan':{'points':[[0,0],[.5,0]]}}
    if costmap_test:
        import copy
        global_grid=copy.deepcopy(grid);global_grid['data'][5*20+5]=253
        local_grid={'frame_id':'odom','width':2,'height':2,'resolution':.1,'data':[254,0,0,0],
                    'origin':{'position':{'x':0,'y':0,'z':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}
        packet['costmaps']={
            'global':{'available':True,'fresh':True,'aligned':True,'age_sec':.1,'revision':1,'grid':global_grid,'transform':{'x':0,'y':0,'yaw':0}},
            'local':{'available':True,'fresh':True,'aligned':True,'age_sec':.1,'revision':1,'grid':local_grid,'transform':{'x':.5,'y':.5,'yaw':math.pi/2}}}
    try:
        async with async_playwright() as pw:
            executable=os.environ.get('COCKPIT_BROWSER_EXE')
            browser=await pw.chromium.launch(headless=True,**({'executable_path':executable} if executable else {}))
            page=await browser.new_page(viewport={'width':1500,'height':1100});page.set_default_timeout(10000)
            page.on('pageerror',lambda e:errors.append(str(e)))
            async def route_http(route):
                nonlocal reject_next
                u=urlparse(route.request.url)
                if u.hostname=='127.0.0.1' and u.port==server.server_port:await route.continue_();return
                if u.port==8000:
                    headers={'Access-Control-Allow-Origin':'*','Access-Control-Allow-Headers':'Content-Type','Access-Control-Allow-Methods':'GET,POST,OPTIONS'}
                    if route.request.method=='OPTIONS':await route.fulfill(status=204,headers=headers);return
                    if route.request.method=='POST':
                        body=route.request.post_data_json;posts.append((u.path,body))
                        if u.path=='/api/localization/initialpose':
                            assert body['confirmed'] is True
                            n['readiness'].update(ready=True,reasons=[])
                        elif u.path=='/api/localization/auto':
                            n['localization'].update(state='WAITING',message='等待激光');n['readiness'].update(ready=False,reasons=['自动定位中'])
                        elif u.path=='/api/localization/cancel':
                            n['localization'].update(state='CANCELED',message='已取消自动定位')
                        elif u.path=='/api/navigation/precheck':
                            valid=not reject_next;reject_next=False
                            await route.fulfill(json={'ok':True,'valid':valid,'path_checked':False,
                               'issues':[] if valid else ['目标 1：太靠近障碍，已进入车体膨胀禁入区（代价 253）'],
                               'targets':[dict(body['waypoints'][0],valid=valid,cost=0 if valid else 253)]},headers=headers);return
                        elif u.path=='/api/navigation/task':
                            assert body['mode']=='single' and len(body['waypoints'])==1
                            n.update(status_code=2,status_text='EXECUTING',paused=False,waypoints=body['waypoints'],
                                total_waypoints=1,distance_remaining=.8,estimated_time_remaining=9,task_id='ui-test')
                        elif u.path=='/api/navigation/pause':n.update(status_code=3,status_text='CANCELING')
                        elif u.path=='/api/navigation/resume':n.update(status_code=2,status_text='EXECUTING',paused=False)
                        elif u.path=='/api/navigation/cancel':n.update(status_code=5,status_text='CANCELED',paused=False)
                        else:raise AssertionError('Unexpected POST '+u.path)
                        await route.fulfill(json={'ok':True,'task_id':'ui-test'},headers=headers);return
                    if u.path=='/api/maps':await route.fulfill(json={'maps':[],'saved_map':{}},headers=headers);return
                await route.abort()
            async def route_ws(ws):
                if ws.url.endswith('/ws/telemetry') or ws.url.endswith('/ws/mapping'):
                    data=health if ws.url.endswith('/ws/telemetry') else packet
                    ws.send(json.dumps(data))
                    async def feed():
                        while True:
                            await asyncio.sleep(.15)
                            try:ws.send(json.dumps(data))
                            except Exception:return
                    tasks.append(asyncio.create_task(feed()))
                elif ws.url.endswith('/ws/control'):
                    ws.send(json.dumps({'type':'ready'}));ws.on_message(lambda d:commands.append(json.loads(d)))
                else:ws.on_message(lambda d:None)
            await page.route('**/*',route_http);await page.route_web_socket('**/*',route_ws)
            await page.goto(f'http://127.0.0.1:{server.server_port}/jetauto_pro_cockpit.html')
            if not auto_test:await page.wait_for_function("!document.querySelector('#btnToolInitPose').disabled")
            else:await page.wait_for_function("!document.querySelector('#btnCancelAutoLocalize').disabled")
            assert await page.locator('#btnToolGoalPose').is_disabled()
            async def drag(tool,dx,offset=(0,0),dy=0):
                await page.locator(tool).click(force=True)
                canvas=page.locator('#mapCanvas2D');await canvas.scroll_into_view_if_needed()
                b=await canvas.bounding_box();x=b['x']+b['width']/2+offset[0];y=b['y']+b['height']/2+offset[1]
                await page.mouse.move(x,y);await page.mouse.down();await page.mouse.move(x+dx,y+dy,steps=5);await page.mouse.up()
            if auto_test:
                assert await page.locator('#btnToolInitPose').is_disabled()
                assert '90°' in await page.locator('#autoLocalizationStatus').inner_text()
                assert not posts,'Automatic positioning must not require a manual pose POST'
                n['localization'].update(state='SUCCEEDED',message='自动定位成功');n['readiness'].update(ready=True,reasons=[])
                await page.wait_for_function("!document.querySelector('#btnAutoLocalize').disabled")
                await page.locator('#btnAutoLocalize').click(force=True)
                await page.wait_for_function("!document.querySelector('#btnCancelAutoLocalize').disabled")
                await page.locator('#btnCancelAutoLocalize').click(force=True)
                n['localization'].update(state='SUCCEEDED',message='自动定位成功');n['readiness'].update(ready=True,reasons=[])
            else:
                await drag('#btnToolInitPose',55)
                assert not posts,'Staging must not publish initialpose'
                assert '待确认' in await page.locator('#initialPoseDraft').inner_text()
                await page.locator('#btnConfirmInitialPose').click(force=True)
            await page.wait_for_function("!document.querySelector('#btnToolGoalPose').disabled")
            if not auto_test:assert len(posts)==1 and posts[0][0]=='/api/localization/initialpose'
            if costmap_test:
                await page.wait_for_function("document.querySelector('#liveCostmapStatus').textContent.includes('全局代价地图：正常')")
                await page.locator('#mapCanvas2D').scroll_into_view_if_needed();await page.wait_for_timeout(150)
                async def pixel(x,y):
                    return await page.locator('#mapCanvas2D').evaluate('''(c,p)=>{
                      const scale=Math.min((c.width-60)/2,(c.height-100)/2);
                      return Array.from(c.getContext('2d').getImageData(Math.round(c.width/2+p[0]*scale),Math.round(c.height/2-p[1]*scale),1,1).data);
                    }''',[x,y])
                orange=await pixel(-.45,-.45);red=await pixel(.45,.55)
                assert orange[0]>200 and 90<orange[1]<190 and orange[2]<150,orange
                assert red[0]>180 and red[1]<100 and red[2]<100,red
                await page.locator('#chkLayerGlobalCostmap').uncheck(force=True);await page.wait_for_timeout(100)
                assert await pixel(-.45,-.45)==[255,255,255,255]
                await page.locator('#chkLayerGlobalCostmap').check(force=True)
                await page.locator('#chkLayerCostmap').uncheck(force=True);await page.wait_for_timeout(100)
                assert await pixel(.45,.55)==[255,255,255,255]
                await page.locator('#chkLayerCostmap').check(force=True)
                packet['costmaps']['global'].update(fresh=False,issue='全局代价地图已过期')
                await page.wait_for_function("document.querySelector('#liveCostmapStatus').textContent.includes('已过期')")
                await page.wait_for_timeout(100);assert await pixel(-.45,-.45)==[255,255,255,255]
                packet['costmaps']['global'].update(fresh=True,issue=None)
                await drag('#btnToolGoalPose',45)
                await page.wait_for_function("document.querySelector('#navPrecheckStatus').textContent.includes('253')")
                assert not any(path=='/api/navigation/task' for path,_ in posts),'Blocked goal must never be submitted'
                assert '未下发' in await page.locator('#navPrecheckStatus').inner_text()
            await drag('#btnToolGoalPose',45,(7,11),13)
            await page.wait_for_function("!document.querySelector('#btnNavPause').disabled")
            target = next(body['waypoints'][0] for path,body in posts if path=='/api/navigation/task')
            assert abs(target['yaw_deg']-math.degrees(math.atan2(-13,45))) < .001, target
            assert posts[-1][0]=='/api/navigation/task'
            assert await page.locator('#btnNavStart').is_disabled()
            await page.locator('#btnNavPause').click(force=True)
            await page.wait_for_timeout(400)
            assert await page.locator('#btnNavResume').is_disabled(),'Wait for cancel acknowledgement'
            n.update(status_code=5,status_text='PAUSED',paused=True)
            await page.wait_for_function("!document.querySelector('#btnNavResume').disabled")
            await page.locator('#btnNavResume').click(force=True)
            await page.wait_for_function("!document.querySelector('#btnNavPause').disabled")
            await page.locator('#btnNavCancel').click(force=True)
            await page.wait_for_function("!document.querySelector('#btnNavStart').disabled")
            expected=['/api/localization/auto','/api/localization/cancel'] if auto_test else ['/api/localization/initialpose']
            if costmap_test:expected+=['/api/navigation/precheck']
            assert [p[0] for p in posts]==expected+['/api/navigation/precheck','/api/navigation/task','/api/navigation/pause','/api/navigation/resume','/api/navigation/cancel']
            assert not commands,commands
            assert not errors,errors
            if os.environ.get('COCKPIT_NAV_SCREENSHOT'):await page.screenshot(path=os.environ['COCKPIT_NAV_SCREENSHOT'],full_page=True)
            await browser.close()
            print('Navigation browser PASS:', 'automatic rotation status, success without manual pose, retry/cancel' if auto_test else 'staged/confirmed initialpose', '; goal, feedback, pause/resume/cancel; zero manual velocity commands.')
    finally:
        for task in tasks:task.cancel()
        await asyncio.gather(*tasks,return_exceptions=True);server.shutdown();server.server_close()

if __name__=='__main__':asyncio.run(main())
