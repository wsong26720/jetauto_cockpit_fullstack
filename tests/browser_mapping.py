"""Read-only mapping UI tests; all robot connections are intercepted."""
import asyncio
import copy
import functools
import http.server
import json
import math
import os
import threading
from pathlib import Path
from urllib.parse import urlparse
from playwright.async_api import async_playwright

ROOT=Path(__file__).resolve().parents[1]


async def main():
    server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(http.server.SimpleHTTPRequestHandler,directory=str(ROOT/'frontend')))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    errors=[]; commands=[]; posts=[]; streams=[]; tasks=[]
    blocked=False
    unavailable=False
    grid={'frame_id':'map','width':2,'height':2,'resolution':1,'data':[0,100,-1,50],
          'origin':{'position':{'x':-1,'y':-1,'z':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}
    packet={'type':'mapping','available':True,'frame_id':'map','map_revision':1,'map_age_sec':0.1,
            'pose':{'x':0,'y':0,'yaw':0,'age_sec':0.02},
            'scan':{'frame_id':'map','source_frame':'lidar_frame','sequence':1,'age_sec':0.02,'points':[[0.5,-0.5]]},
            'tf':{'robot':True,'scan':True,'issues':[]}}
    telemetry={'control':{},'capabilities':{'mapping_view':True},'control_ready':False,'estop_engaged':False,
               'hardware':{},'launches':[],'control_issues':['测试：只读显示']}
    try:
        async with async_playwright() as pw:
            executable=os.environ.get('COCKPIT_BROWSER_EXE')
            browser=await pw.chromium.launch(headless=True,**({'executable_path':executable} if executable else {}))
            page=await browser.new_page(viewport={'width':1400,'height':1000})
            page.set_default_timeout(10000)
            page.on('pageerror',lambda err:errors.append(str(err)))
            async def route_http(route):
                if route.request.method=='POST': posts.append(route.request.url)
                u=urlparse(route.request.url)
                if u.hostname=='127.0.0.1' and u.port==server.server_port:
                    await route.continue_();return
                if u.port==8000:
                    headers={'Access-Control-Allow-Origin':'*','Access-Control-Allow-Headers':'Content-Type','Access-Control-Allow-Methods':'GET, POST, OPTIONS'}
                    if route.request.method=='OPTIONS':await route.fulfill(status=204,headers=headers);return
                    if u.path=='/api/maps':
                        await route.fulfill(json={'maps':[{'name':'map_501'}],'saved_map':{'running':False,'active':False}},headers=headers);return
                    if u.path=='/api/map/load':
                        assert route.request.post_data_json['name']=='map_501'
                        for ws in streams:ws.send(json.dumps({**packet,'map':grid}))
                        await route.fulfill(json={'ok':True,'name':'map_501','state':'active'},headers=headers);return
                await route.abort()
            async def route_ws(ws):
                if ws.url.endswith('/ws/mapping'):
                    streams.append(ws)
                    ws.send(json.dumps({'type':'mapping','available':False,'detail':'缺少 tf2_ros'} if unavailable else
                        {'type':'mapping','available':True,'frame_id':'map','map_revision':0,'pose':None,'scan':None,
                         'tf':{'robot':False,'scan':False,'issues':['等待 /map；请加载已保存地图']}}))
                    async def feed_map():
                        while not blocked:
                            await asyncio.sleep(0.1)
                            if blocked: return
                            try:ws.send(json.dumps(packet))
                            except Exception:return
                    if not unavailable:tasks.append(asyncio.create_task(feed_map()))
                elif ws.url.endswith('/ws/telemetry'):
                    ws.send(json.dumps(telemetry))
                    async def feed_health():
                        while True:
                            await asyncio.sleep(0.5)
                            try:ws.send(json.dumps(telemetry))
                            except Exception:return
                    tasks.append(asyncio.create_task(feed_health()))
                elif ws.url.endswith('/ws/control'):
                    ws.send(json.dumps({'type':'ready'}))
                    ws.on_message(lambda data:commands.append(json.loads(data)))
                else: ws.on_message(lambda data:None)
            await page.route('**/*',route_http)
            await page.route_web_socket('**/*',route_ws)
            await page.goto(f'http://127.0.0.1:{server.server_port}/jetauto_pro_cockpit.html')
            await page.wait_for_function("!document.querySelector('#btnLoadSavedMap').disabled")
            assert '等待' in await page.locator('#liveMapStatus').inner_text()
            await page.locator('#inputNavMapPath').fill('map_501')
            await page.locator('#btnLoadSavedMap').click(force=True)
            await page.wait_for_function("document.querySelector('#liveMapStatus').textContent.includes('2 × 2')")
            await page.wait_for_function("document.querySelector('#liveTfStatus').textContent.includes('激光 TF：正常')")
            await page.wait_for_timeout(100)  # Status updates before the next 20Hz canvas frame.
            canvas=page.locator('#mapCanvas2D')
            async def pixel(dx,dy):
                return await canvas.evaluate('''(c,p)=>Array.from(c.getContext('2d').getImageData(
                    Math.round(c.width/2+p[0]),Math.round(c.height/2+p[1]),1,1).data)''',[dx,dy])
            # Fit scale is 165px/m for this 2m x 2m map in a 430px viewport.
            assert await pixel(-82,-82)==[205,205,205,255]  # upper-left unknown
            assert await pixel(82,-82)==[128,128,128,255]   # upper-right 50% occupancy
            assert await pixel(-82,82)==[255,255,255,255]   # bottom-left free
            assert await pixel(82,82)==[239,68,68,255]      # TF-aligned scan over black cell
            assert await pixel(-20,0)==[37,99,235,255]      # robot at map origin
            assert await page.locator('#hudPoseX').inner_text()=='0.00'
            await page.locator('#chkLayerScan').uncheck(force=True)
            await page.wait_for_timeout(100)
            assert await pixel(82,82)==[0,0,0,255]
            await page.locator('#chkLayerScan').check(force=True)
            await page.locator('#chkLayerMap').uncheck(force=True)
            await page.wait_for_timeout(100)
            assert await pixel(-82,-82)==[226,232,240,255]
            await page.locator('#chkLayerMap').check(force=True)
            await page.wait_for_timeout(100)
            image=await canvas.evaluate('c=>c.toDataURL()')
            await canvas.hover(position={'x':300,'y':200})
            await page.mouse.wheel(0,-200)
            await page.wait_for_timeout(100)
            assert image!=await canvas.evaluate('c=>c.toDataURL()')
            await page.locator('#btnFitMap').click(force=True)
            await page.wait_for_timeout(100)
            assert image==await canvas.evaluate('c=>c.toDataURL()')
            # A rotated OccupancyGrid must rotate with its origin orientation.
            grid=copy.deepcopy(grid)
            grid['origin']={'position':{'x':0,'y':0,'z':0},'orientation':{'x':0,'y':0,'z':math.sqrt(0.5),'w':math.sqrt(0.5)}}
            packet['map_revision']=2
            for ws in streams:ws.send(json.dumps({**packet,'map':grid}))
            await page.wait_for_timeout(100)
            assert await pixel(82,82)==[255,255,255,255]
            assert await pixel(82,-82)==[0,0,0,255]
            packet['tf']={'robot':False,'scan':False,'issues':['缺少 map → lidar_frame TF']}
            packet['pose']=None;packet['scan']=None
            for ws in streams:ws.send(json.dumps(packet))
            await page.wait_for_function("document.querySelector('#hudPoseX').textContent==='--'")
            assert '缺少' in await page.locator('#liveMapStatus').inner_text()
            # Optional visual QA with the real robot sample captured by a read-only audit.
            fixture=os.environ.get('COCKPIT_MAP_FIXTURE')
            screenshot=os.environ.get('COCKPIT_MAP_SCREENSHOT')
            if fixture and screenshot:
                sample=json.loads(Path(fixture).read_text(encoding='utf-8'))
                grid=sample.pop('map');packet.clear();packet.update(sample)
                for ws in streams:ws.send(json.dumps({**packet,'map':grid}))
                await page.wait_for_function("document.querySelector('#liveMapStatus').textContent.includes('65 × 100')")
                await page.locator('#btnFitMap').click(force=True)
                await page.wait_for_timeout(150)
                await page.locator('#mapViewport').screenshot(path=screenshot)
            blocked=True
            for ws in streams:await ws.close()
            await page.wait_for_function("document.querySelector('#liveMapStatus').textContent.includes('断开')")
            assert '等待/过期' in await page.locator('#liveTfStatus').inner_text()
            unavailable=True
            await page.locator('#btnModeSim').click(force=True)
            await page.locator('#btnModeLive').click(force=True)
            await page.wait_for_function("document.querySelector('#liveMapStatus').textContent.includes('缺少 tf2_ros')")
            assert not commands,commands
            assert len(posts)==1 and posts[0].endswith('/api/map/load'),posts
            assert not errors,errors
            await browser.close()
            print('Mapping browser OK: saved-map load API, grid pixels/orientation, laser/pose, layers, zoom/fit, TF/disconnect; zero velocity commands, only map-load POST.')
    finally:
        for task in tasks:task.cancel()
        await asyncio.gather(*tasks,return_exceptions=True)
        server.shutdown();server.server_close()


if __name__=='__main__':asyncio.run(main())
