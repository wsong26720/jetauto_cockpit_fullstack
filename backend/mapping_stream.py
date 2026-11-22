"""Read-only map/laser feed. ROS imports are delayed until a node is available."""
import math
import threading
import time
from collections import deque


def xyz(v):
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def quaternion(v):
    return {**xyz(v), "w": float(v.w)}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def valid_rotation_norm(translation, rotation):
    coordinates = (translation.x, translation.y, translation.z)
    norm = math.hypot(rotation.x, rotation.y, rotation.z, rotation.w)
    if not all(math.isfinite(v) for v in coordinates) or not math.isfinite(norm) or abs(norm-1) > .01:
        raise ValueError("TF 位移或四元数无效")
    return norm


def scan_points(scan, transform):
    """Project valid rays using the full 3D lidar-to-map rigid transform.

    Roll/pitch must not be discarded: some mounted lidars are inverted.
    """
    t, q = transform.translation, transform.rotation
    norm = valid_rotation_norm(t, q)
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    r00, r01 = 1-2*(y*y+z*z), 2*(x*y-z*w)
    r10, r11 = 2*(x*y+z*w), 1-2*(x*x+z*z)
    points = []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance <= 0 or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        sx, sy = distance * math.cos(angle), distance * math.sin(angle)
        px, py = t.x + r00*sx + r01*sy, t.y + r10*sx + r11*sy
        if math.isfinite(px) and math.isfinite(py):
            points.append([round(px, 4), round(py, 4)])
    return points


class MappingFeed:
    def __init__(self, node, clock=time.monotonic):
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener, TransformException
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import LaserScan

        self.node, self.clock = node, clock
        self.Time, self.TransformException = Time, TransformException
        self.lock = threading.RLock()
        self.grid = None
        self.revision = 0
        self.map_received = None
        self.scan_received = None
        self.map_error = None
        self.scan_sequence = 0
        self.scans = deque(maxlen=30)
        self.plan = None
        self.base_frame = "base_footprint"
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        # A late browser/gateway must receive the existing latched map/static TF.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = node.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.plan_sub = node.create_subscription(Path,"/plan",self.on_plan,10)
        from costmap_stream import CostmapFeed
        self.costmap_error=None
        try:self.costmaps=CostmapFeed(node,clock)
        except ImportError as exc:self.costmaps=None;self.costmap_error='代价地图组件不可用：'+str(exc)

    def on_plan(self,msg):
        count=len(msg.poses);step=max(1,math.ceil(count/5000))
        points=[[float(p.pose.position.x),float(p.pose.position.y)] for p in msg.poses[::step]
                if math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)]
        with self.lock:self.plan={'frame_id':msg.header.frame_id.lstrip('/'),'points':points,'received':self.clock()}

    def on_map(self, msg):
        info = msg.info
        size = info.width * info.height
        p, q = info.origin.position, info.origin.orientation
        try:
            valid_rotation_norm(p, q)
            origin_valid = True
        except ValueError:
            origin_valid = False
        if not size or size > 16_000_000 or size != len(msg.data) or not math.isfinite(info.resolution) or info.resolution <= 0 or not origin_valid:
            with self.lock:
                self.map_error = "地图原点、尺寸或分辨率无效，或超过 1600 万格显示上限"
            return
        frame = msg.header.frame_id.lstrip("/")
        if not frame:
            with self.lock:
                self.map_error = "地图 frame_id 为空"
            return
        grid = {"frame_id": frame, "stamp": stamp_seconds(msg.header.stamp),
                "width": info.width, "height": info.height, "resolution": info.resolution,
                "origin": {"position": xyz(info.origin.position), "orientation": quaternion(info.origin.orientation)},
                "data": list(msg.data)}
        with self.lock:
            previous = self.grid
            if previous and all(previous[k] == grid[k] for k in ('frame_id','width','height','resolution','origin','data')):
                self.map_received = self.clock()
                self.map_error = None
                return
            self.grid = grid
            self.revision += 1
            self.map_received = self.clock()
            self.map_error = None
            if getattr(self,'costmaps',None):self.costmaps.reset()

    def on_scan(self, msg):
        with self.lock:
            self.scan_sequence += 1
            self.scan_received = self.clock()
            self.scans.append((self.scan_sequence, self.scan_received, msg))

    def localization_input(self):
        """Map and scan in base coordinates, independent of the missing map TF."""
        with self.lock:
            if self.map_error:
                return None
            grid=self.grid;revision=self.revision;scans=list(self.scans)
        if not grid or self.node.count_publishers('/map')!=1:return None
        now=self.clock();ros_now=self.node.get_clock().now().nanoseconds*1e-9
        for sequence,received,scan in reversed(scans):
            if now-received>1:break
            if not 0<=ros_now-stamp_seconds(scan.header.stamp)<1:continue
            try:
                at=self.Time.from_msg(scan.header.stamp)
                t=self.buffer.lookup_transform(self.base_frame,scan.header.frame_id.lstrip('/'),at)
                o=self.buffer.lookup_transform('odom',self.base_frame,at).transform
                q=o.rotation
                valid_rotation_norm(o.translation, q)
                return {'grid':grid,'revision':revision,'sequence':sequence,
                        'points':scan_points(scan,t.transform),
                        'odom':{'x':o.translation.x,'y':o.translation.y,
                                'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}}
            except (self.TransformException,ValueError):continue
        return None

    def snapshot(self, last_revision=None, costmap_revisions=None, *, include_scan_points=True):
        now = self.clock()
        with self.lock:
            grid, revision = self.grid, self.revision
            scans = list(self.scans)
            map_received, scan_received, map_error = self.map_received, self.scan_received, self.map_error
            plan=getattr(self,'plan',None)
        frame = grid["frame_id"] if grid else "map"
        issues = []
        payload = {"type": "mapping", "available": True, "frame_id": frame,
                   "map_revision": revision, "map_age_sec": None if map_received is None else round(now-map_received, 2),
                   "scan_age_sec": None if scan_received is None else round(now-scan_received, 2),
                   "pose": None, "scan": None, "tf": {"robot": False, "scan": False, "issues": issues}}
        payload['plan']={'frame_id':frame,'points':plan['points'],'age_sec':round(now-plan['received'],3)} if plan and plan['frame_id']==frame and now-plan['received']<3 else None
        def resolve(source):
            tf=self.buffer.lookup_transform(frame,source,self.Time())
            stamp=stamp_seconds(tf.header.stamp)
            ros_now=self.node.get_clock().now().nanoseconds*1e-9
            if stamp and not -1.1 <= ros_now-stamp <= 2:
                raise ValueError(frame+' ← '+source+' TF 已过期或时钟不同步')
            t=tf.transform.translation;q=tf.transform.rotation
            valid_rotation_norm(t, q)
            return {'x':t.x,'y':t.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
        costs=getattr(self,'costmaps',None)
        payload['costmaps']=costs.snapshot(frame,resolve,costmap_revisions) if costs else {
            name:{'available':False,'fresh':False,'aligned':False,'issue':getattr(self,'costmap_error',None) or '等待代价地图组件','revision':0}
            for name in ('global','local')}
        if grid is not None and revision != last_revision:
            payload["map"] = grid
        if grid is None:
            issues.append(map_error or "等待 /map；请启动 SLAM 或加载已保存地图")
            return payload
        if map_error:
            issues.append(map_error)
        publishers = self.node.count_publishers('/map')
        payload['map_valid'] = publishers == 1 and not map_error
        if publishers == 0:
            issues.append('当前没有 /map 发布者，底图为最后收到的地图')
        elif publishers > 1:
            issues.append('存在多个 /map 发布者，请选择唯一地图源')
        try:
            tf = self.buffer.lookup_transform(frame, self.base_frame, self.Time())
            tf_stamp = stamp_seconds(tf.header.stamp)
            ros_now = self.node.get_clock().now().nanoseconds * 1e-9
            # Static transforms can have stamp 0; live robot pose must be dynamic.
            age = ros_now - tf_stamp
            if tf_stamp <= 0 or not -1.1 <= age <= 2:
                issues.append("机器人 TF 已过期，等待实时 " + frame + " → " + self.base_frame)
            else:
                t, q = tf.transform.translation, tf.transform.rotation
                valid_rotation_norm(t, q)
                payload["pose"] = {"x": t.x, "y": t.y,
                    "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                    "stamp": tf_stamp, "age_sec": round(age, 3)}
                payload["tf"]["robot"] = True
        except self.TransformException:
            issues.append("缺少机器人 TF：" + frame + " → " + self.base_frame)
        except ValueError as exc:
            issues.append(str(exc))
        if not scans or now - scans[-1][1] > 2:
            issues.append("等待 /scan" if not scans else "激光扫描已过期")
            return payload
        scan_error = None
        # The newest scan may be ahead of TF by one update. Retry recent scans
        # rather than silently applying a transform from the wrong timestamp.
        for sequence, received, scan in reversed(scans):
            if now - received > 1:
                break
            source = scan.header.frame_id.lstrip("/")
            if not source:
                scan_error = "激光 frame_id 为空"
                continue
            try:
                stamp = stamp_seconds(scan.header.stamp)
                acquisition_age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
                if stamp <= 0 or not -.1 <= acquisition_age <= 1:
                    scan_error = '激光采集时间已过期或时钟不同步'
                    continue
                tf = self.buffer.lookup_transform(frame, source, self.Time.from_msg(scan.header.stamp))
                if include_scan_points:
                    payload["scan"] = {"sequence": sequence, "source_frame": source, "frame_id": frame,
                                       "stamp": stamp, "age_sec": round(max(acquisition_age, now-received), 3),
                                       "points": scan_points(scan, tf.transform)}
                else:
                    valid_rotation_norm(tf.transform.translation, tf.transform.rotation)
                payload["tf"]["scan"] = True
                return payload
            except self.TransformException:
                scan_error = "等待扫描时刻 TF：" + frame + " → " + source
            except ValueError as exc:
                scan_error = str(exc)
        issues.append(scan_error or "没有可用的激光时间同步数据")
        return payload
