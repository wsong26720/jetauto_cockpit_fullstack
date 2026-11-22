"""Nav2 task orchestration, confirmed localization and readiness checks."""
import copy
import math
import threading
import time
import uuid
import os
from fastapi import HTTPException
from automatic_localization import AutomaticLocalization
from navigation_precheck import precheck


ACTIVE=('map_server','amcl','controller_server','planner_server','smoother_server',
        'behavior_server','bt_navigator','waypoint_follower','velocity_smoother')


def localization_issue(amcl, robot_pose, now):
    covariance = amcl['covariance']
    if len(covariance) != 36 or not all(math.isfinite(v) for v in covariance):
        return 'AMCL 协方差无效'
    xx, xy, yy, yaw_variance = (covariance[i] for i in (0, 1, 7, 35))
    if min(xx, yy, yaw_variance) < 0 or xx * yy + 1e-9 < xy * xy:
        return 'AMCL 协方差无效'
    largest = (xx + yy + math.hypot(xx-yy, 2*xy)) / 2
    if largest > .5 or yaw_variance > .5:
        return '定位不确定度较高，请重新确认初始位姿'
    # AMCL can legitimately stay silent at rest.
    if robot_pose and now - amcl['received'] > 3:
        delta = robot_pose['yaw'] - math.radians(amcl['yaw_deg'])
        if (math.hypot(robot_pose['x']-amcl['x'], robot_pose['y']-amcl['y']) > .10 or
                abs(math.atan2(math.sin(delta), math.cos(delta))) > .12):
            return '机器人已移动，等待新的 AMCL 定位观测'
    return None


class NavigationBridge:
    def __init__(self,node,mapping_getter,motion,hardware_ready):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose,NavigateThroughPoses
        from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
        from lifecycle_msgs.srv import GetState
        from rclpy.qos import qos_profile_sensor_data
        from std_srvs.srv import Empty
        self.node=node;self.mapping_getter=mapping_getter;self.motion=motion;self.hardware_ready=hardware_ready
        self.PoseStamped=PoseStamped;self.InitialPose=PoseWithCovarianceStamped
        self.actions={'single':NavigateToPose,'through_poses':NavigateThroughPoses}
        self.clients={'single':ActionClient(node,NavigateToPose,'/navigate_to_pose'),
                      'through_poses':ActionClient(node,NavigateThroughPoses,'/navigate_through_poses')}
        self.pose_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.Empty=Empty;self.nomotion_client=node.create_client(Empty,'/request_nomotion_update')
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self._on_pose,qos_profile_sensor_data)
        self.GetState=GetState
        self.lifecycle={name:node.create_client(GetState,'/'+name+'/get_state') for name in ACTIVE}
        self.states={};self.state_futures={};self.last_poll=0
        self.lock=threading.RLock();self.confirmed=False;self.initial_stamp=0;self.amcl=None
        self.ready_cache={'ready':False,'reasons':['等待导航节点与初始位姿'],'lifecycle':{}}
        self.ready_at=0;self.task=None;self.thread=None;self.handle=None
        self.stop_event=threading.Event();self.stop_reason=None;self.unresolved=False
        self.phase='IDLE';self.code=0;self.index=0;self.loop=1;self.dwell_remaining=0
        self.metrics={'distance_remaining':0.0,'estimated_time_remaining':0.0,'navigation_time':0.0,'number_of_recoveries':0}
        self.result_status=None;self.error=None
        self.goal_generation=0
        self.initial_source=None
        self.manual_good_samples = 0
        self.manual_seed = None
        self.manual_seed_agrees = False
        self.manual_settled = False
        self.manual_update = None
        self.manual_update_sent = 0.
        self.automatic=AutomaticLocalization(self,enabled=os.environ.get('JETAUTO_AUTO_LOCALIZATION','true').lower() not in ('false','0','no'))
        node.create_timer(0.2,self._tick)

    def _on_pose(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        with self.lock:
            if self.initial_stamp and stamp>=self.initial_stamp and msg.header.frame_id.lstrip('/')=='map':
                if self.amcl and stamp <= self.amcl['stamp']:
                    return
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(math.hypot(q.x,q.y,q.z,q.w)-1) > .01:
                    self.amcl = None
                    self.ready_cache['ready'] = False
                    return
                previous = self.amcl
                self.amcl={'stamp':stamp,'covariance':list(msg.pose.covariance),'received':time.monotonic(),
                           'x':p.x,'y':p.y,'yaw_deg':math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))}
                if self.initial_source == 'manual' and not self.manual_settled:
                    stable = False
                    seed = self.manual_seed
                    seed_angle = math.radians(self.amcl['yaw_deg'] - seed['yaw_deg'])
                    self.manual_seed_agrees = (
                        math.hypot(p.x-seed['x'], p.y-seed['y']) <= .20
                        and abs(math.atan2(math.sin(seed_angle), math.cos(seed_angle))) <= math.radians(12)
                    )
                    if self.manual_seed_agrees and previous and localization_issue(self.amcl, None, time.monotonic()) is None:
                        delta = math.radians(self.amcl['yaw_deg'] - previous['yaw_deg'])
                        stable = (math.hypot(p.x-previous['x'], p.y-previous['y']) <= .05
                                  and abs(math.atan2(math.sin(delta), math.cos(delta))) <= math.radians(3))
                    self.manual_good_samples = self.manual_good_samples + 1 if stable else 0
                    self.manual_settled = self.manual_good_samples >= 3

    def _state_reply(self,name,future):
        with self.lock:
            if self.state_futures.get(name,(None,))[0] is not future:
                return
            try:
                response=future.result()
                self.states[name]=(response.current_state.id,time.monotonic()) if response else (0,time.monotonic())
            except Exception:self.states[name]=(0,time.monotonic())
            if self.state_futures.get(name,(None,))[0] is future:self.state_futures.pop(name,None)

    def _tick(self):
        now=time.monotonic()
        if now-self.last_poll>=1:
            self.last_poll=now
            for name,client in self.lifecycle.items():
                with self.lock:
                    pending=self.state_futures.get(name)
                    if pending:
                        if now-pending[1]>2:pending[0].cancel();self.state_futures.pop(name,None)
                        else:continue
                if client.service_is_ready():
                    f=client.call_async(self.GetState.Request())
                    with self.lock:self.state_futures[name]=(f,now)
                    f.add_done_callback(lambda result,n=name:self._state_reply(n,result))
                else:
                    with self.lock:self.states[name]=(0,now)
        reasons=[]
        with self.lock:
            states={name:self.states.get(name,(0,0))[0] if now-self.states.get(name,(0,0))[1]<4 else 0 for name in ACTIVE}
        mapping=self.mapping_getter()
        sample=mapping.snapshot(last_revision=mapping.revision, include_scan_points=False) if mapping else None
        if self.automatic.armed and states.get('amcl')==3 and not self.motion.manual.estop:
            try:self.automatic.start()
            except HTTPException:pass
        localization=self.automatic.snapshot()
        with self.lock:
            confirmed, amcl, initial_stamp = self.confirmed, self.amcl, self.initial_stamp
            seed_agrees = self.manual_seed_agrees
            settling = self.initial_source == 'manual' and confirmed and not self.manual_settled
            if settling and states.get('amcl') == 3 and now-self.manual_update_sent >= .5:
                if self.manual_update is not None and not self.manual_update.done() and now-self.manual_update_sent > 2:
                    self.manual_update.cancel()
                    self.manual_update = None
                if self.nomotion_client.service_is_ready() and (self.manual_update is None or self.manual_update.done()):
                    self.manual_update = self.nomotion_client.call_async(self.Empty.Request())
                    self.manual_update_sent = now
        if settling:
            if amcl and not seed_agrees:
                reasons.append('AMCL 与人工标注相差超过 0.20 m 或 12°，请核对地图、位置和朝向')
            else:
                reasons.append('人工初始位姿已提交，等待 AMCL 连续 3 次稳定更新')
        if not sample or sample.get('map_revision',0)==0:reasons.append('等待地图 /map')
        elif not sample.get('map_valid', False):reasons.append('需要唯一且有效的 /map 地图源')
        if not confirmed:reasons.append(localization['message'] if localization['enabled'] else '请在地图上确认初始位置与朝向')
        elif not amcl:reasons.append('等待 AMCL 响应初始位姿')
        else:
            issue = localization_issue(amcl, sample.get('pose') if sample else None, now)
            if issue:
                reasons.append(issue)
        if not sample or not sample['tf']['robot']:reasons.append('等待 map → base_footprint TF')
        if not sample or not sample['tf']['scan']:reasons.append('等待激光与地图 TF 对齐')
        if not self.hardware_ready():reasons.append('底盘控制链未就绪')
        inactive=[name for name,state in states.items() if state!=3]
        if inactive:reasons.append('导航生命周期未激活：'+', '.join(inactive))
        if not self.clients['single'].server_is_ready():reasons.append('导航 Action 服务未就绪')
        publishers=self.node.get_publishers_info_by_topic('/cmd_vel')
        unexpected=[p.node_name for p in publishers if p.node_name!='jetauto_web_cockpit_gateway']
        if unexpected:reasons.append('/cmd_vel 存在未经过网关的发布源：'+', '.join(sorted(set(unexpected))))
        if self.node.count_publishers('/cockpit/nav_cmd_vel')==0:reasons.append('导航速度未接入网关私有通道')
        costs=getattr(mapping,'costmaps',None)
        if not costs:reasons.append('等待导航代价地图组件')
        else:
            _,cost_error=costs.raw('global');_,profile_error=costs.footprint()
            if cost_error:reasons.append(cost_error)
            if profile_error:reasons.append(profile_error)
        if self.motion.manual.estop:reasons.append('急停已锁定')
        with self.lock:
            if initial_stamp != self.initial_stamp or confirmed != self.confirmed or amcl is not self.amcl:
                return
            self.ready_cache={'ready':not reasons,'reasons':reasons,'lifecycle':states,
                              'initial_pose_confirmed':confirmed,'amcl_received':amcl is not None,
                              'initial_pose_source':self.initial_source,
                              'amcl_covariance':amcl['covariance'] if amcl else None}
            self.ready_at=now

    def motion_ready(self):
        auto=getattr(self,'automatic',None)
        return bool(self.confirmed and not(auto and auto.active()) and time.monotonic()-self.ready_at<0.7 and self.ready_cache.get('ready'))

    def _point(self,point):
        values=[float(point.get(k,0)) for k in ('x','y','yaw_deg','dwell_sec')]
        if not all(math.isfinite(x) for x in values):raise HTTPException(422,'目标坐标必须是有限数值')
        if not 0<=values[3]<=300:raise HTTPException(422,'驻留时间应在 0–300 秒之间')
        return dict(point,x=values[0],y=values[1],yaw_deg=values[2],dwell_sec=values[3])

    def _validate_map_point(self,point):
        feed=self.mapping_getter()
        if not feed or not feed.grid:raise HTTPException(503,'尚未收到地图')
        grid=feed.grid;o=grid['origin'];q=o['orientation'];p=o['position']
        yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
        dx,dy=point['x']-p['x'],point['y']-p['y']
        gx=math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/grid['resolution'])
        gy=math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/grid['resolution'])
        if not 0<=gx<grid['width'] or not 0<=gy<grid['height']:raise HTTPException(422,'所选位置超出地图范围')
        cell=grid['data'][gy*grid['width']+gx]
        if cell<0 or cell>=65:raise HTTPException(422,'请在地图白色可通行区域选择位置')

    def initial_pose(self,point,confirmed):
        if not confirmed:raise HTTPException(422,'需要确认机器人实际位置与朝向')
        p=self._point(point);self._validate_map_point(p)
        with self.lock:
            if self.automatic.active():raise HTTPException(409,'自动定位正在运行，请先取消自动定位再手动标注')
            if self.thread and self.thread.is_alive():raise HTTPException(409,'请先取消导航，再设置初始位姿')
            if self.motion.manual.owner:raise HTTPException(409,'请先松开手动控制器')
            if self.states.get('amcl',(0,))[0]!=3:raise HTTPException(503,'AMCL 尚未激活，请先启动导航模块')
            self.automatic.cancel('已切换为可选手动定位')
            self._publish_initial(p,source='manual')
            self.confirmed=True
        return {'ok':True,'state':'waiting_for_amcl','frame_id':'map'}

    def _publish_initial(self,p,source):
        msg=self.InitialPose();msg.header.frame_id='map';msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x=float(p['x']);msg.pose.pose.position.y=float(p['y'])
        yaw=math.radians(p['yaw_deg']);msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        variance_xy=float(p.get('seed_variance_xy',.04)) if source=='auto_pending' else .25
        variance_yaw=float(p.get('seed_variance_yaw',math.radians(10)**2)) if source=='auto_pending' else math.radians(15)**2
        msg.pose.covariance[0]=variance_xy;msg.pose.covariance[7]=variance_xy;msg.pose.covariance[35]=variance_yaw
        self.initial_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.initial_source=source;self.amcl=None;self.ready_cache['ready']=False
        self.manual_seed = dict(p) if source == 'manual' else None
        self.manual_seed_agrees = False
        self.manual_good_samples = 0
        self.manual_settled = False
        self.manual_update_sent = 0.
        self.pose_pub.publish(msg)

    def invalidate(self,reset_unresolved=False):
        self.automatic.cancel('导航上下文已改变')
        self.cancel('context_changed')
        with self.lock:
            self.confirmed=False;self.amcl=None;self.initial_stamp=0
            if reset_unresolved:self.unresolved=False
            self.ready_cache['ready']=False

    def cancel_automatic(self):
        was_active=self.automatic.active()
        self.automatic.cancel()
        with self.lock:
            if was_active:self.confirmed=False;self.ready_cache['ready']=False
        return {'ok':True,'state':'CANCELED'}

    def _check_start(self):
        if self.motion.manual.estop:raise HTTPException(423,'急停已锁定')
        if self.unresolved:raise HTTPException(409,'上一目标接收确认超时，请重启导航模块后重试')
        if not self.motion_ready():raise HTTPException(409,'；'.join(self.ready_cache['reasons']) or '导航尚未就绪')

    def start(self,mode,points,loop_count=1):
        if mode not in ('single','through_poses','waypoints'):raise HTTPException(422,'未知导航模式')
        if not 1<=len(points)<=100 or not 1<=loop_count<=10:raise HTTPException(422,'站点数量应为 1–100，循环次数为 1–10')
        points=[self._point(p) for p in points]
        if mode=='single' and len(points)!=1:raise HTTPException(422,'单点导航只接受一个目标')
        for p in points:self._validate_map_point(p)
        with self.lock:
            self._check_start()
            if self.thread and self.thread.is_alive():raise HTTPException(409,'已有导航任务，请先暂停或取消')
            if self.phase == 'PAUSED':
                raise HTTPException(409, '已有暂停的任务，请先恢复或取消')
            if self.motion.manual.owner:raise HTTPException(409,'请先松开手动控制器')
            self._require_precheck(points)
            self.task={'id':uuid.uuid4().hex,'mode':mode,'points':points,'loops':loop_count}
            self.index=0;self.loop=1;self.dwell_remaining=0;self.error=None;self.result_status=None
            self.metrics={'distance_remaining':0.0,'estimated_time_remaining':0.0,'navigation_time':0.0,'number_of_recoveries':0}
            self._launch_thread()
            return {'ok':True,'task_id':self.task['id'],'state':'pending'}

    def _launch_thread(self):
        self.stop_event=threading.Event();self.stop_reason=None;self.phase='PENDING';self.code=1
        self.result_status=None
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def _pose(self,point):
        msg=self.PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.position.x=point['x'];msg.pose.position.y=point['y']
        yaw=math.radians(point['yaw_deg']);msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
        return msg

    @staticmethod
    def _wait(future,timeout):
        event=threading.Event();future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout):raise TimeoutError('Nav2 响应超时')
        return future.result()

    def _feedback(self,event,token,generation):
        feedback=event.feedback
        with self.lock:
            if not self.task or self.task['id']!=token or self.goal_generation!=generation:return
            for name in ('distance_remaining','number_of_recoveries'):
                value=getattr(feedback,name,0)
                if math.isfinite(float(value)):self.metrics[name]=value
            for name in ('estimated_time_remaining','navigation_time'):
                duration=getattr(feedback,name,None)
                if duration:self.metrics[name]=duration.sec+duration.nanosec*1e-9
            remaining=getattr(feedback,'number_of_poses_remaining',None)
            if remaining is not None:self.index=max(0,min(len(self.task['points'])-1,len(self.task['points'])-remaining))

    def _execute(self,points,batch):
        if self.stop_event.is_set():return None
        self._require_precheck(points)
        mode='through_poses' if batch else 'single'
        action=self.actions[mode];goal=action.Goal()
        if batch:goal.poses=[self._pose(p) for p in points]
        else:goal.pose=self._pose(points[0])
        token=self.task['id']
        with self.lock:
            if self.stop_event.is_set():return None
            self.goal_generation+=1;generation=self.goal_generation
            future=self.clients[mode].send_goal_async(goal,feedback_callback=lambda event:self._feedback(event,token,generation))
        try:
            handle=self._wait(future,5)
        except TimeoutError:
            with self.lock:self.unresolved=True
            def late_reply(f):
                try:
                    h=f.result()
                    if h and h.accepted:h.cancel_goal_async()
                except Exception:pass
            future.add_done_callback(late_reply)
            raise
        if not handle or not handle.accepted:raise RuntimeError('Nav2 拒绝了导航目标')
        with self.lock:
            self.handle=handle
            if not self.stop_event.is_set():
                try:self.motion.begin(token)
                except Exception:
                    handle.cancel_goal_async();raise
            self.phase='EXECUTING';self.code=2
        result=handle.get_result_async();cancel_sent=False;cancel_at=0;lost_at=0
        while not result.done():
            if not self.motion_ready():
                if not lost_at:lost_at=time.monotonic()
                if time.monotonic()-lost_at>1 and not self.stop_event.is_set():self.cancel('localization_or_readiness_lost')
            else:lost_at=0
            if self.stop_event.is_set() and not cancel_sent:
                handle.cancel_goal_async();cancel_sent=True;cancel_at=time.monotonic()
            if cancel_sent and time.monotonic()-cancel_at>5:
                with self.lock:self.unresolved=True
                raise TimeoutError('取消目标后未收到终态，请重启导航模块')
            time.sleep(0.05)
        response=result.result()
        with self.lock:self.handle=None;self.result_status=response.status
        return response.status

    def _run(self):
        token=self.task['id']
        try:
            while self.loop<=self.task['loops']:
                batch=self.task['mode']=='through_poses'
                while self.index<len(self.task['points']):
                    if self.stop_event.is_set():break
                    if self.dwell_remaining<=0:
                        points=self.task['points'][self.index:] if batch else [self.task['points'][self.index]]
                        result=self._execute(points,batch)
                        self.motion.end(token)
                        if self.stop_event.is_set():break
                        if result!=4:
                            report=self.precheck(points)
                            reason='；'.join(report['issues']) if not report['valid'] else '当前起终点碰撞检查通过，请查看 Nav2 规划/控制日志'
                            raise RuntimeError('Nav2 中止了导航（ROS 状态 '+str(result)+'）：'+reason)
                        if batch:self.index=len(self.task['points']);break
                        self.dwell_remaining=self.task['points'][self.index]['dwell_sec'] if self.task['mode']=='waypoints' else 0
                    with self.lock:self.phase='DWELLING' if self.dwell_remaining else 'EXECUTING'
                    last=time.monotonic()
                    while self.dwell_remaining>0 and not self.stop_event.wait(0.05):
                        now=time.monotonic();self.dwell_remaining=max(0,self.dwell_remaining-(now-last));last=now
                    if self.stop_event.is_set():break
                    self.index+=1
                if self.stop_event.is_set():break
                self.loop+=1
                if self.loop<=self.task['loops']:self.index=0
            with self.lock:
                if self.stop_event.is_set():
                    if self.stop_reason=='pause':self.phase='PAUSED';self.code=5
                    elif self.stop_reason in ('cancel','manual_stop','estop','context_changed','shutdown'):self.phase='CANCELED';self.code=5
                    else:self.phase='ABORTED';self.code=6;self.error=self.stop_reason
                else:
                    self.phase='SUCCEEDED';self.code=4
                    self.metrics['distance_remaining']=0.0
                    self.metrics['estimated_time_remaining']=0.0
        except Exception as exc:
            with self.lock:self.error=str(exc.detail) if isinstance(exc,HTTPException) else str(exc);self.phase='ABORTED';self.code=6
        finally:self.motion.end(token)

    def cancel(self,reason='cancel'):
        with self.lock:
            self.stop_reason=reason;self.stop_event.set()
            self.motion.end(self.task['id'] if self.task else None)
            if self.thread and self.thread.is_alive():self.phase='CANCELING';self.code=3
            elif self.phase=='PAUSED':self.phase='CANCELED';self.code=5
            return {'ok':True,'state':self.phase,'task_id':self.task['id'] if self.task else None}

    def resume(self):
        with self.lock:
            self._check_start()
            if self.phase!='PAUSED' or (self.thread and self.thread.is_alive()):raise HTTPException(409,'任务尚未进入可恢复的暂停状态')
            self._require_precheck(self.task['points'][self.index:])
            self._launch_thread()
            return {'ok':True,'state':'pending','task_id':self.task['id']}

    def precheck(self,points):
        if not 1<=len(points)<=100:raise HTTPException(422,'预检查点数应在 1–100 之间')
        points=[self._point(p) for p in points]
        return precheck(self.mapping_getter(),points)

    def _require_precheck(self,points):
        report=self.precheck(points)
        if not report['valid']:raise HTTPException(422,'导航目标预检查失败：'+'；'.join(report['issues']))
        return report

    def status(self):
        with self.lock:
            ready=copy.deepcopy(self.ready_cache)
            task=self.task
            return {'status_code':self.code,'status_text':self.phase,'task_id':task['id'] if task else None,
                'mode':task['mode'] if task else 'single','current_waypoint_index':min(self.index,max(0,len(task['points'])-1)) if task else 0,
                'total_waypoints':len(task['points']) if task else 0,'current_loop':min(self.loop,task['loops']) if task else 1,
                'paused':self.phase=='PAUSED','dwell_remaining':round(self.dwell_remaining,1),
                'error':self.error,'ros_result_status':self.result_status,'readiness':ready,
                'localization':self.automatic.snapshot(),
                'waypoints':copy.deepcopy(task['points']) if task else [],**self.metrics}
