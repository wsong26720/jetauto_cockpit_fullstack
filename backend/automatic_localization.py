"""Stationary startup localization: global scan search then AMCL verification."""
import copy
import math
import threading
import time
import os
from fastapi import HTTPException


BUSY=('WAITING','MATCHING','ROTATING','VERIFYING')


class AutomaticLocalization:
    def __init__(self,bridge,enabled=True):
        self.bridge=bridge;self.enabled=enabled;self.armed=enabled
        self.lock=threading.RLock();self.thread=None;self.stop=threading.Event();self.started=0
        self.state='IDLE';self.message='启动导航后自动扫描定位';self.match=None;self.error=None
        self.rotation_allowed=os.environ.get('JETAUTO_AUTO_ROTATE','false').lower() in ('true','1','yes')
        self.rotation_progress=0.;self.rotation_checked=0.
        bridge.motion.localization_ready=lambda:bool(self.state=='ROTATING' and time.monotonic()-self.rotation_checked<.3 and not self.stop.is_set())

    def snapshot(self):
        with self.lock:
            return {'enabled':self.enabled,'state':self.state,'message':self.message,'error':self.error,
                    'elapsed_sec':round(time.monotonic()-self.started,1) if self.state in BUSY else 0,
                    'match':copy.deepcopy(self.match),'rotation_allowed':self.rotation_allowed,
                    'rotation_degrees':round(math.degrees(self.rotation_progress),1),'stationary':not self.rotation_allowed}

    def active(self):
        return self.state in BUSY

    def arm(self):
        with self.lock:
            self.armed=self.enabled

    def cancel(self,reason='自动定位已取消'):
        with self.lock:
            self.armed=False;self.stop.set()
            if self.active():self.state='CANCELED';self.message=reason

    def start(self):
        b=self.bridge
        with b.lock:
            if b.thread and b.thread.is_alive():raise HTTPException(409,'请先取消导航任务，再自动定位')
            if b.motion.manual.owner:raise HTTPException(409,'请先松开手动控制器')
            if b.motion.manual.estop:raise HTTPException(423,'请先解除急停，再自动定位')
            with self.lock:
                if self.thread and self.thread.is_alive():raise HTTPException(409,'自动定位尚在处理中')
                self.armed=False;self.started=time.monotonic();self.state='WAITING'
                self.message='正在等待地图、激光与底盘 TF；请保持机器人静止'
                self.error=None;self.match=None;self.stop=threading.Event();self.rotation_progress=0
                b.confirmed=False;b.amcl=None;b.initial_stamp=0;b.ready_cache['ready']=False
                b.initial_source=None
                b.motion.end();b.motion.manual.stop_manual()
                self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
        return {'ok':True,'state':'WAITING','rotation_allowed':self.rotation_allowed,'stationary':not self.rotation_allowed}

    def _inputs(self):
        b=self.bridge
        if b.motion.manual.estop:raise RuntimeError('急停已锁定，自动定位停止')
        feed=b.mapping_getter()
        if not feed or b.states.get('amcl',(0,))[0]!=3:return None
        return feed.localization_input()

    @staticmethod
    def _stationary(start,current):
        from global_scan_matcher import angle_delta
        a=start['odom'];c=current['odom']
        return math.hypot(a['x']-c['x'],a['y']-c['y'])<=.05 and abs(angle_delta(a['yaw'],c['yaw']))<=math.radians(5)

    @staticmethod
    def _pose_good(pose,seed,quality):
        from global_scan_matcher import angle_delta
        if not pose:return False
        cov=pose['covariance'];xymax=(cov[0]+cov[7]+math.sqrt((cov[0]-cov[7])**2+4*cov[1]**2))/2
        return (all(math.isfinite(v) for v in cov) and cov[0]>=0 and cov[7]>=0 and cov[35]>=0
                and cov[0]*cov[7]>=cov[1]**2-1e-9 and xymax<=.04 and cov[35]<=math.radians(10)**2
                and math.hypot(pose['x']-seed['x'],pose['y']-seed['y'])<=.25
                and abs(angle_delta(math.radians(pose['yaw_deg']-seed['yaw_deg']),0))<=math.radians(12)
                and quality['hit_ratio']>=.65 and quality['free_ratio']>=.95)

    @staticmethod
    def _rotation_clear(points):
        # Ignore returns inside the configured body, check the swept body radius.
        for x,y in points:
            if abs(x)<=.27 and abs(y)<=.24:continue
            if math.hypot(x,y)<.43:return False
        return True

    def _collect_rotation(self,anchor,deadline):
        from global_scan_matcher import angle_delta
        b=self.bridge;token='localization:'+str(self.started);combined=list(anchor['points'])
        previous_yaw=anchor['odom']['yaw'];last_saved=0.;last_sequence=anchor['sequence']
        until=min(deadline,time.monotonic()+15)
        while not b.hardware_ready() and time.monotonic()<until and not self.stop.is_set():
            with self.lock:self.state='WAITING';self.message='正在等待底盘驱动和有效电池采样，再执行定位旋转'
            self.stop.wait(.2)
        if self.stop.is_set():raise InterruptedError('自动定位已取消')
        if not b.hardware_ready():raise RuntimeError('底盘或电池未就绪，不能执行自动定位旋转')
        current=self._inputs()
        if not current or not self._stationary(anchor,current):raise RuntimeError('等待驱动过程中机器人移动或传感器中断，请重试')
        unexpected=[p.node_name for p in b.node.get_publishers_info_by_topic('/cmd_vel') if p.node_name!='jetauto_web_cockpit_gateway']
        if unexpected:raise RuntimeError('检测到其他底盘速度发布者，不能安全执行自动定位旋转')
        if not self._rotation_clear(anchor['points']):raise RuntimeError('旋转范围 0.43m 内检测到障碍，保持停车')
        with self.lock:self.state='ROTATING';self.message='静止扫描存在相似候选，正在以 0.10rad/s 原地旋转采集周围环境'
        try:
            b.motion.begin(token,localization=True)
            while not self.stop.wait(.1) and time.monotonic()<deadline:
                current=self._inputs()
                if not current:raise RuntimeError('旋转时激光或底盘 TF 中断，已停车')
                if current['revision']!=anchor['revision']:raise RuntimeError('旋转时地图改变，已停车')
                if not b.hardware_ready():raise RuntimeError('旋转时底盘或电池未就绪，已停车')
                if not self._rotation_clear(current['points']):raise RuntimeError('旋转范围内检测到障碍，已停车')
                if math.hypot(current['odom']['x']-anchor['odom']['x'],current['odom']['y']-anchor['odom']['y'])>.10:
                    raise RuntimeError('原地旋转出现超过 0.10m 的平移，已停车')
                if current['sequence']!=last_sequence:
                    delta=angle_delta(current['odom']['yaw'],previous_yaw)
                    if abs(delta)>.15:raise RuntimeError('旋转时里程计朝向跳变，已停车')
                    self.rotation_progress+=delta;previous_yaw=current['odom']['yaw'];last_sequence=current['sequence']
                    if self.rotation_progress<-.1:raise RuntimeError('旋转方向与指令不一致，已停车')
                    if self.rotation_progress-last_saved>=.15:
                        a=angle_delta(current['odom']['yaw'],anchor['odom']['yaw']);ca=math.cos(a);sa=math.sin(a)
                        dx=current['odom']['x']-anchor['odom']['x'];dy=current['odom']['y']-anchor['odom']['y']
                        c=math.cos(anchor['odom']['yaw']);s=math.sin(anchor['odom']['yaw'])
                        tx=c*dx+s*dy;ty=-s*dx+c*dy
                        combined.extend([[ca*x-sa*y+tx,sa*x+ca*y+ty] for x,y in current['points']])
                        last_saved=self.rotation_progress
                self.rotation_checked=time.monotonic()
                if self.rotation_progress>=2*math.pi-.03:return combined,current
                b.motion.localization_command(token,.10)
            if self.stop.is_set():raise InterruptedError('自动定位已取消')
            raise TimeoutError('原地旋转未在限定时间内完成，已停车')
        finally:
            self.rotation_checked=0;b.motion.end(token)

    def _run(self):
        b=self.bridge;deadline=self.started+(150 if self.rotation_allowed else 60);last_service=0;future=None
        try:
            while not self.stop.is_set() and time.monotonic()<deadline:
                start=self._inputs()
                if start:break
                self.stop.wait(.2)
            else:
                if self.stop.is_set():return
                raise TimeoutError('限定时间内未等到地图、AMCL、激光或底盘 TF')
            from global_scan_matcher import GlobalScanMatcher
            with self.lock:self.state='MATCHING';self.message='正在整张地图搜索位置与朝向，不驱动车轮'
            matcher=GlobalScanMatcher(start['grid']);match=matcher.match(start['points'],self.stop.is_set)
            if not match['accepted'] and self.rotation_allowed:
                merged,end=self._collect_rotation(start,min(deadline,time.monotonic()+85))
                with self.lock:self.state='MATCHING';self.message='旋转采集完成，正在融合扫描并重新搜索整张地图'
                match=matcher.match(merged,self.stop.is_set)
                if match['accepted']:
                    from global_scan_matcher import angle_delta
                    c=math.cos(start['odom']['yaw']);s=math.sin(start['odom']['yaw'])
                    dx=end['odom']['x']-start['odom']['x'];dy=end['odom']['y']-start['odom']['y']
                    tx=c*dx+s*dy;ty=-s*dx+c*dy;yaw=math.radians(match['yaw_deg'])
                    match['x']+=math.cos(yaw)*tx-math.sin(yaw)*ty
                    match['y']+=math.sin(yaw)*tx+math.cos(yaw)*ty
                    match['yaw_deg']=math.degrees(angle_delta(yaw+end['odom']['yaw']-start['odom']['yaw'],0))
                start=end
            with self.lock:self.match=match
            if not match['accepted']:raise RuntimeError(match['reason']+'；保持停车，可更新地图或调整环境后重试')
            current=self._inputs()
            if not current or current['revision']!=start['revision'] or not self._stationary(start,current):
                raise RuntimeError('搜索过程中地图改变、传感器中断或机器人移动，请保持静止后重试')
            with b.lock:
                if self.stop.is_set():return
                b._publish_initial(match,source='auto_pending')
            with self.lock:self.state='VERIFYING';self.message='已找到候选位置，正在验证 AMCL、扫描匹配和稳定性'
            good=[];last_stamp=0;last_sequence=0;stable_start=0
            while not self.stop.wait(.2) and time.monotonic()<deadline:
                current=self._inputs()
                if current and (current['revision']!=start['revision'] or not self._stationary(start,current)):
                    raise RuntimeError('验证过程中地图改变或机器人移动，自动定位停止')
                now=time.monotonic()
                if now-last_service>=.5 and b.nomotion_client.service_is_ready() and (future is None or future.done()):
                    future=b.nomotion_client.call_async(b.Empty.Request());last_service=now
                with b.lock:pose=copy.deepcopy(b.amcl)
                if not current or not pose or now-pose['received']>2:
                    good=[];stable_start=0;continue
                if pose['stamp']==last_stamp or current['sequence']==last_sequence:continue
                last_stamp=pose['stamp'];last_sequence=current['sequence']
                quality=matcher.assess(pose,current['points'])
                sample=b.mapping_getter().snapshot(last_revision=current['revision'])
                if not self._pose_good(pose,match,quality) or not sample['tf']['robot'] or not sample['tf']['scan']:
                    good=[];stable_start=0;continue
                if good and (math.hypot(pose['x']-good[0]['x'],pose['y']-good[0]['y'])>.10 or
                             abs(math.atan2(math.sin(math.radians(pose['yaw_deg']-good[0]['yaw_deg'])),math.cos(math.radians(pose['yaw_deg']-good[0]['yaw_deg']))))>math.radians(5)):
                    good=[];stable_start=0
                if not good:stable_start=now
                good.append(pose)
                if len(good)>=8 and now-stable_start>=3:
                    with b.lock:
                        if self.stop.is_set():return
                        b.confirmed=True;b.initial_source='automatic'
                    with self.lock:
                        self.state='SUCCEEDED';self.message='自动定位成功；等待导航节点与底盘就绪'
                        self.match.update(verified_pose={k:pose[k] for k in ('x','y','yaw_deg')},verified_quality=quality)
                    return
            if not self.stop.is_set():raise TimeoutError('自动定位未在限定时间内可靠收敛；保持停车，请检查地图和雷达后重试')
        except Exception as exc:
            if not self.stop.is_set():
                with b.lock:b.confirmed=False;b.ready_cache['ready']=False
                with self.lock:self.state='FAILED';self.message=str(exc);self.error=str(exc)
