#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetAuto Pro ROS 2 Humble Web 数字驾驶舱 — 车端网关守护服务 (FastAPI + rclpy)
目标硬件: NVIDIA Jetson Orin NX Super (Ubuntu 22.04, ROS 2 Humble)
默认监听: 0.0.0.0:8000 (本机检查地址 192.168.3.73)

提供硬件遥测、单客户端手动控制租约、软急停及受管 Launch 启停。
提供确认初始位姿、Nav2 任务编排和统一速度仲裁。
"""
import asyncio
import json
import math
import os
import signal
import shlex
import re
import uuid
import tempfile
from contextlib import asynccontextmanager, nullcontext
from control_safety import VelocityLease, MotionArbiter, ControlError
from navigation_bridge import NavigationBridge
from mapping_stream import MappingFeed
from saved_maps import SavedMapManager
from costmap_settings import CostmapSettingsStore, RuntimeCostmapSettings, SettingsError, differences
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import psutil
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from std_msgs.msg import UInt16
    from std_srvs.srv import Trigger
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

app = FastAPI(
    title="JetAuto Pro ROS 2 Humble Cockpit Gateway",
    version="2.0",
    description="车端网关服务：硬件遥测、手动/导航速度仲裁、初始定位、Nav2 任务与 Launch 管理",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

WORKSPACE_DIR = os.environ.get("JETAUTO_WS", "/home/ubuntu/ros2_ws")
THIRD_PARTY_WS_DIR = os.environ.get("THIRD_PARTY_WS", "/home/ubuntu/third_party/third_party_ws")
WAYPOINTS_FILE = Path(__file__).parent / "waypoints_store.json"

SCRIPT_DIR = Path(__file__).resolve().parent
ROS_ENV_PREFIX = "source " + shlex.quote(str(SCRIPT_DIR / "ros_env.sh"))
costmap_settings_store = CostmapSettingsStore(SCRIPT_DIR / 'config/nav2_params.yaml')



# ==================== 数据模型定义 ====================

class TwistCommand(BaseModel):
    client_id: str = Field(..., min_length=1, max_length=100, description="每个 HTTP 控制客户端使用独立 ID")
    linear_x: float = Field(0.0, ge=-0.20, le=0.20, description="车体前后线速度 m/s (限幅 ±0.20)")
    linear_y: float = Field(0.0, ge=-0.20, le=0.20, description="麦轮横移线速度 m/s (限幅 ±0.20)")
    angular_z: float = Field(0.0, ge=-0.50, le=0.50, description="自转角速度 rad/s (限幅 ±0.50)")


class WaypointItem(BaseModel):
    id: str
    name: str
    x: float
    y: float
    yaw_deg: float
    dwell_sec: float = 0.0


class NavTaskRequest(BaseModel):
    mode: str = Field(..., description="single | through_poses | waypoints")
    waypoints: List[WaypointItem]
    loop_count: int = 1


class LaunchActionRequest(BaseModel):
    module_id: str
    action: str = Field(..., description="start | stop | restart")
    map_path: Optional[str] = Field(None, description="保存地图名，例如 map_501；不是绝对路径")


class MapSaveRequest(BaseModel):
    output_path: str = Field(..., description="map_saver_cli 输出完整路径（不含扩展名）")


class CostmapSettingsRequest(BaseModel):
    revision: str = Field(..., min_length=64, max_length=64)
    settings: dict


# ==================== Launch 进程组管理器 ====================

LAUNCH_MODULES = {
    "controller": {
        "name": "底盘运动与 EKF 控制栈",
        "cmd": "ros2 launch controller controller.launch.py",
        "conflicts_with": ["slam", "navigation"],
        "category": "driver",
    },
    "depth_camera": {
        "name": "Orbbec Astra Pro Plus 深度相机",
        "cmd": "ros2 launch peripherals depth_camera.launch.py",
        "conflicts_with": ["slam", "navigation"],
        "category": "sensor",
    },
    "lidar": {
        "name": "YDLIDAR G4 激光雷达与滤波链",
        "cmd": "ros2 launch peripherals lidar.launch.py",
        "conflicts_with": ["slam", "navigation"],
        "category": "sensor",
    },
    "slam": {
        "name": "2D SLAM 在线建图",
        "cmd": "ros2 launch slam slam.launch.py",
        "conflicts_with": ["navigation", "controller", "depth_camera", "lidar"],
        "category": "navigation",
    },
    "navigation": {
        "name": "Nav2 自主定位与导航",
        "cmd_template": "ros2 launch "+shlex.quote(str(SCRIPT_DIR/"cockpit_navigation.launch.py"))+" map:={map_path}",
        "default_map": "map_501",
        "conflicts_with": ["slam", "controller", "depth_camera", "lidar"],
        "category": "navigation",
    },
    "web_video_server": {
        "name": "Web 视频流服务 (:8080)",
        "cmd": "ros2 run web_video_server web_video_server --ros-args -p address:=0.0.0.0 -p port:=8080",
        "conflicts_with": [],
        "category": "web",
    },
}


class ProcessManager:
    LAUNCHES = {
        "controller": ("controller", "controller.launch.py"),
        "depth_camera": ("peripherals", "depth_camera.launch.py"),
        "lidar": ("peripherals", "lidar.launch.py"),
        "slam": ("slam", "slam.launch.py"),
        "navigation": ("navigation", "navigation.launch.py"),
        "ros_robot_controller": ("ros_robot_controller", "ros_robot_controller.launch.py"),
    }
    EXECUTABLES = {
        "controller": {"odom_publisher", "odom_publisher_node", "ros_robot_controller", "ros_robot_controller_node"},
        "depth_camera": {"astra_camera_node", "astra_camera_node_factory", "camera_node", "usb_cam_node_exe"},
        "lidar": {"ydlidar_ros2_driver_node"},
        "slam": {"async_slam_toolbox_node", "sync_slam_toolbox_node"},
        "navigation": {"controller_server", "planner_server", "bt_navigator"},
        "web_video_server": {"web_video_server"},
    }

    def __init__(self):
        self.processes = {}
        self.logs = {k: [] for k in LAUNCH_MODULES}
        self.start_times = {}
        self.custom_args = {}
        self.lock = threading.RLock()
        self.stopping = set()

    def _append_log(self, module_id, line):
        with self.lock:
            buf = self.logs.setdefault(module_id, [])
            buf.append(time.strftime("[%H:%M:%S] ") + line.rstrip()[:4000])
            del buf[:-200]

    def _reader_thread(self, module_id, proc):
        for raw in iter(proc.stdout.readline, b""):
            self._append_log(module_id, raw.decode("utf-8", errors="replace"))

    def is_running(self, module_id):
        proc = self.processes.get(module_id)
        return proc is not None and proc.poll() is None

    def discover(self):
        # Detect host-local launchers and drivers, including ones outside this gateway.
        # Conservative host-wide detection avoids reopening a serial/USB device even
        # if an existing driver belongs to another ROS domain. Never kill these PIDs.
        found = {k: [] for k in LAUNCH_MODULES}
        found["ros_robot_controller"] = []
        for proc in psutil.process_iter(["pid", "cmdline"]):
            try:
                args = proc.info["cmdline"] or []
                words = [Path(v).name for v in args]
                for key, pair in self.LAUNCHES.items():
                    if any(args[i:i+3] == ["launch", *pair] for i in range(len(args)-2)):
                        found[key].append(proc.info["pid"])
                for key, executables in self.EXECUTABLES.items():
                    if any(w in executables for w in words[:2]):
                        found[key].append(proc.info["pid"])
                if "__node:=nav2_container" in args:
                    found["navigation"].append(proc.info["pid"])
                if any(args[i]=='launch' and args[i+1].endswith('cockpit_navigation.launch.py') for i in range(len(args)-1)):
                    found['navigation'].append(proc.info['pid'])
            except (psutil.Error, OSError):
                continue
        return found

    def start(self, module_id, map_path=None):
        with self.lock:
            if module_id not in LAUNCH_MODULES:
                raise HTTPException(400, "未知模块")
            spec = LAUNCH_MODULES[module_id]
            if module_id in self.stopping:raise HTTPException(409,'模块正在停止，请稍后重试')
            if self.is_running(module_id):
                return {"status": "already_running", "pid": self.processes[module_id].pid}
            found = self.discover()
            if found[module_id]:
                raise HTTPException(409, "检测到外部同类进程，先检查并手动处理；网关不会接管或终止外部进程")
            conflicts = list(spec.get("conflicts_with", []))
            if module_id == "controller":
                conflicts.append("ros_robot_controller")
            for conflict in conflicts:
                if self.is_running(conflict) or found.get(conflict):
                    raise HTTPException(409, "请先停止冲突模块：" + conflict)
            devices = []
            if module_id in ("controller", "slam", "navigation"):
                devices.append("/dev/rrc")
            if module_id in ("lidar", "slam", "navigation"):
                devices.append("/dev/lidar")
            missing = [p for p in devices if not Path(p).exists()]
            if missing:
                raise HTTPException(503, "设备未识别：" + ", ".join(missing) + "；请检查底盘/雷达供电与 USB 连接")
            if module_id == "navigation":
                name = (map_path or spec["default_map"]).strip()
                if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
                    raise HTTPException(422, "厂商 navigation 的 map 参数仅接受地图名，例如 map_01")
                map_file=Path(os.environ.get("JETAUTO_MAP_DIR",str(Path(WORKSPACE_DIR)/"src/slam/maps")))/(name+".yaml")
                if not map_file.is_file():raise HTTPException(404,"地图 YAML 不存在："+str(map_file))
                cmd = spec["cmd_template"].format(map_path=shlex.quote(str(map_file)))
                self.custom_args[module_id] = name
            else:
                cmd = spec["cmd"]
            if module_id == "navigation":
                try:
                    from ament_index_python.packages import get_package_share_directory
                except ImportError as exc:
                    raise HTTPException(503,"未加载 ROS 包索引环境") from exc
                missing=[]
                for package in ("nav2_map_server","nav2_amcl","nav2_lifecycle_manager","nav2_controller",
                                "nav2_planner","nav2_bt_navigator","nav2_theta_star_planner","nav2_smoother",
                                "nav2_velocity_smoother","nav2_waypoint_follower","nav2_behaviors","dwb_critics"):
                    try:get_package_share_directory(package)
                    except LookupError:missing.append(package)
                if missing:
                    raise HTTPException(503,"Nav2 依赖不完整："+", ".join(missing)+"；查看保存地图请使用独立的加载地图按钮")
            self._append_log(module_id, "启动请求: " + cmd)
            try:
                proc = subprocess.Popen(["/bin/bash", "-c", f"{ROS_ENV_PREFIX} && exec {cmd}"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                raise HTTPException(503, str(exc)) from exc
            self.processes[module_id] = proc
            self.start_times[module_id] = time.time()
            threading.Thread(target=self._reader_thread, args=(module_id, proc), daemon=True).start()
            return {"status": "starting", "pid": proc.pid, "cmd": cmd}

    def stop(self, module_id):
        with self.lock:
            if module_id not in LAUNCH_MODULES:
                raise HTTPException(400, "未知模块")
            if not self.is_running(module_id):
                if self.discover().get(module_id):
                    raise HTTPException(409, "外部进程不受本网关管理，请在原启动终端停止")
                return {"status": "stopped"}
            proc = self.processes[module_id]
            if module_id in self.stopping:raise HTTPException(409,'模块正在停止')
            self.stopping.add(module_id)
        # Drain stdout on the reader thread while launch waits for children to exit.
        try:
            owned_group=proc.pid  # start_new_session=True makes PID the session/group ID.
            try:
                os.killpg(os.getpgid(proc.pid),signal.SIGINT);proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid),signal.SIGTERM)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired as exc:raise HTTPException(504,'进程仍未退出，请检查日志') from exc
            except ProcessLookupError:pass
            # ROS launch may exit before a vendor Python child leaves the session.
            # Only clean up the process group created for this managed launch.
            try:
                os.killpg(owned_group,0)
            except ProcessLookupError:pass
            else:
                os.killpg(owned_group,signal.SIGTERM)
                until=time.monotonic()+2
                while time.monotonic()<until:
                    try:os.killpg(owned_group,0)
                    except ProcessLookupError:break
                    time.sleep(.1)
                else:
                    os.killpg(owned_group,signal.SIGKILL)
                    self._append_log(module_id,'受管子进程未响应退出信号，已终止该启动器专属进程组')
            self._append_log(module_id,'受管进程已退出')
            return {'status':'stopped'}
        finally:
            with self.lock:self.stopping.discard(module_id)

    def status_all(self):
        found = self.discover()
        with self.lock:
            items = []
            for key, spec in LAUNCH_MODULES.items():
                managed = self.is_running(key)
                proc = self.processes.get(key)
                external = not managed and bool(found[key])
                items.append({"id": key, "name": spec["name"], "category": spec["category"],
                    "cmd": spec.get("cmd", spec.get("cmd_template", "")),
                    "running": managed or external, "managed": managed, "external": external,
                    "pid": proc.pid if managed else (found[key][0] if external else None),
                    "state": "managed_process" if managed else ("external_process" if external else "stopped"),
                    "exit_code": proc.poll() if proc else None,
                    "uptime_sec": round(time.time()-self.start_times[key], 1) if managed else 0,
                    "conflicts_with": spec.get("conflicts_with", []),
                    "map_path": self.custom_args.get(key, spec.get("default_map", "")),
                    "recent_logs": list(self.logs[key][-30:])})
            return items


process_mgr = ProcessManager()


# ==================== Jetson Orin NX Super 硬件状态采集 ====================

def read_jetson_thermal() -> Dict[str, Optional[float]]:
    temps: Dict[str, Optional[float]] = {"cpu_temp_c": None, "gpu_temp_c": None, "soc_temp_c": None}
    thermal_base = Path("/sys/devices/virtual/thermal")
    if not thermal_base.exists():
        return temps
    for zone in thermal_base.glob("thermal_zone*"):
        try:
            ztype = (zone / "type").read_text().strip().lower()
            raw_temp = float((zone / "temp").read_text().strip())
            val_c = round(raw_temp / 1000.0 if raw_temp > 200 else raw_temp, 1)
            if "cpu" in ztype and temps["cpu_temp_c"] is None:
                temps["cpu_temp_c"] = val_c
            elif "gpu" in ztype and temps["gpu_temp_c"] is None:
                temps["gpu_temp_c"] = val_c
            elif "soc" in ztype and temps["soc_temp_c"] is None:
                temps["soc_temp_c"] = val_c
        except Exception:
            continue
    return temps


def read_jetson_gpu_load() -> float:
    candidate_paths = [
        "/sys/devices/gpu.0/load",
        "/sys/devices/platform/gpu.0/load",
        "/sys/devices/17000000.ga10b/load",
    ]
    for p in candidate_paths:
        if os.path.exists(p):
            try:
                val = float(Path(p).read_text().strip())
                return round(val / 10.0, 1)
            except Exception:
                pass
    return 0.0


def collect_hardware_metrics(battery_mv: Optional[int], battery_ts: Optional[float]) -> dict:
    vm = psutil.virtual_memory()
    sm = psutil.swap_memory()
    du = psutil.disk_usage("/")
    temps = read_jetson_thermal()
    cpu_per_core = psutil.cpu_percent(interval=None, percpu=True)
    cpu_avg = round(sum(cpu_per_core) / max(len(cpu_per_core), 1), 1)

    battery_v = round(battery_mv / 1000.0, 2) if battery_mv is not None else None
    battery_age_sec = round(time.time() - battery_ts, 1) if battery_ts else None

    return {
        "timestamp": time.time(),
        "board_model": "NVIDIA Jetson Orin NX Super (L4T R36.4.3)",
        "cpu_percent": cpu_avg,
        "cpu_cores": cpu_per_core,
        "gpu_percent": read_jetson_gpu_load(),
        "ram_used_gb": round((vm.total - vm.available) / (1024 ** 3), 2),
        "ram_total_gb": round(vm.total / (1024 ** 3), 2),
        "ram_percent": vm.percent,
        "swap_percent": sm.percent,
        "disk_used_gb": round(du.used / (1024 ** 3), 1),
        "disk_total_gb": round(du.total / (1024 ** 3), 1),
        "disk_percent": du.percent,
        "cpu_temp_c": temps["cpu_temp_c"],
        "gpu_temp_c": temps["gpu_temp_c"],
        "battery_voltage_v": battery_v,
        "battery_sample_age_sec": battery_age_sec,
    }


# ==================== ROS 2 Humble 桥接节点 ====================

class JetAutoCockpitNode:
    def __init__(self):
        self._node = None
        self._cmd_vel_pub = None
        self.battery_mv = None
        self.battery_ts = None
        self.init_error = None
        self.mapping = None
        self.mapping_error = None
        self.costmap_parameters = None
        self.costmap_parameters_error = None
        self._mapping_node = None
        self._mapping_executor = None
        self.navigation = None
        self.navigation_error = None
        self._navigation_node = None
        self._navigation_executor = None
        self.nav_state = {"status_code": 0, "status_text": "UNIMPLEMENTED"}
        self.control = VelocityLease(self._publish_raw_twist, self.control_ready)
        self.motion = MotionArbiter(self.control,self._publish_raw_twist,self.control_ready)

    @property
    def estop_engaged(self):
        return self.control.snapshot()["estop_engaged"]

    def start(self):
        if not ROS2_AVAILABLE:
            self.init_error = "缺少 rclpy 或基础 ROS 消息依赖"
            return
        try:
            rclpy.init(args=None,signal_handler_options=SignalHandlerOptions.NO)
            self._node = rclpy.create_node("jetauto_web_cockpit_gateway")
            self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
            self._node.create_subscription(UInt16, "/ros_robot_controller/battery", self._on_battery, 10)
            self._node.create_timer(0.05, self.motion.tick)
            self._node.create_subscription(Twist,"/cockpit/nav_cmd_vel",
                lambda msg:self.motion.receive(msg.linear.x,msg.linear.y,msg.angular.z),10)
            try:
                from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
                self._mapping_node = rclpy.create_node("jetauto_web_mapping")
                self.mapping = MappingFeed(self._mapping_node)
                try:
                    self.costmap_parameters = RuntimeCostmapSettings(self._mapping_node)
                except Exception as exc:
                    self.costmap_parameters_error = str(exc)
                self._mapping_executor = SingleThreadedExecutor()
                self._mapping_executor.add_node(self._mapping_node)
                def spin_mapping():
                    try:
                        self._mapping_executor.spin()
                    except ExternalShutdownException:
                        pass
                threading.Thread(target=spin_mapping, daemon=True).start()
            except Exception as exc:
                self.mapping = None
                self.mapping_error = str(exc)
                if self._mapping_node:
                    self._mapping_node.destroy_node()
                    self._mapping_node = None
            try:
                from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
                self._navigation_node=rclpy.create_node("jetauto_web_navigation")
                self.navigation=NavigationBridge(self._navigation_node,lambda:self.mapping,self.motion,self.control_ready)
                self.motion.nav_ready=self.navigation.motion_ready
                self._navigation_executor=SingleThreadedExecutor();self._navigation_executor.add_node(self._navigation_node)
                def spin_navigation():
                    try:self._navigation_executor.spin()
                    except ExternalShutdownException:pass
                threading.Thread(target=spin_navigation,daemon=True).start()
            except Exception as exc:
                self.navigation=None;self.navigation_error=str(exc)
                if self._navigation_node:self._navigation_node.destroy_node();self._navigation_node=None
            threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()
        except Exception as exc:
            self.init_error = str(exc)

    def control_ready(self):
        return bool(self._node and self.init_error is None and self._cmd_vel_pub
                    and self._cmd_vel_pub.get_subscription_count() > 0
                    and self._node.count_publishers("/ros_robot_controller/battery") > 0
                    and Path("/dev/rrc").exists()
                    and self.battery_ts and time.time()-self.battery_ts < 5
                    and self.battery_mv is not None and self.battery_mv >= 10600)

    def _on_battery(self, msg):
        self.battery_mv = int(msg.data)
        self.battery_ts = time.time()

    def _publish_raw_twist(self, x, y, yaw):
        if self._cmd_vel_pub:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = float(x), float(y), float(yaw)
            self._cmd_vel_pub.publish(msg)

    def send_cmd_vel(self, cmd, owner=None):
        nav = self.navigation
        moving = any((cmd.linear_x, cmd.linear_y, cmd.angular_z))
        # Use the same lock order as NavigationBridge.start: navigation, then
        # the manual lease. The ownership check and first command are atomic.
        with nav.lock if nav else nullcontext():
            if moving and nav and nav.automatic.active():
                raise HTTPException(409,'自动定位中请保持静止；需要手动驾驶时先取消自动定位')
            if nav and nav.thread and nav.thread.is_alive():
                if moving:
                    raise HTTPException(409,'导航任务正在执行，请先暂停或取消后手动接管')
                nav.cancel('manual_stop')
            try:
                self.control.command_from(owner or cmd.client_id, cmd.linear_x, cmd.linear_y, cmd.angular_z)
            except ControlError as exc:
                raise HTTPException(exc.status, exc.detail) from exc

    def trigger_estop(self, engage):
        self.control.set_estop(engage)
        if engage and self.navigation:self.navigation.cancel_automatic()
        if engage and self.navigation:self.navigation.cancel('estop')
        # Only the gateway's manual control is arbitrated here. External ROS
        # publishers/robot applications require a vehicle-wide mux/hardware stop.

    def graph(self):
        if not self._node:
            return {"nodes": [], "cmd_vel_subscribers": 0, "rgb_publishers": 0, "scan_publishers": 0}
        return {"nodes": sorted(self._node.get_node_names()),
            "cmd_vel_subscribers": self._node.count_subscribers("/cmd_vel"),
            "rgb_publishers": self._node.count_publishers("/depth_cam/rgb/image_raw"),
            "scan_publishers": self._node.count_publishers("/scan")}

    def readiness_issues(self):
        issues = []
        if self._node is None or self.init_error:
            issues.append("网关 ROS 节点未就绪")
        if not Path("/dev/rrc").exists():
            issues.append("底盘设备 /dev/rrc 不存在，检查 USB 与供电")
        if not self._cmd_vel_pub or not self._cmd_vel_pub.get_subscription_count():
            issues.append("/cmd_vel 没有订阅者，请启动完整底盘控制链")
        if self._node and self._node.count_publishers("/ros_robot_controller/battery") == 0:
            issues.append("底层控制板节点未发布电池话题，不能仅凭 odom 节点判断底盘就绪")
        if not self.battery_ts or time.time()-self.battery_ts >= 5:
            issues.append("电池遥测未收到或超过 5 秒，暂时禁止驱动")
        elif self.battery_mv is not None and self.battery_mv < 10600:
            issues.append(f"电池电压 {self.battery_mv/1000:.2f}V 低于 10.6V，暂时禁止驱动")
        if self.estop_engaged:
            issues.append("手动急停已锁定，需手动解除")
        return issues

    def shutdown(self):
        self.control.set_estop(True)
        if self.navigation:
            self.navigation.automatic.cancel('网关正在退出')
            self.navigation.cancel('shutdown')
        if self._navigation_executor:self._navigation_executor.shutdown(timeout_sec=1)
        if self._navigation_node:self._navigation_node.destroy_node()
        if self._mapping_executor:
            self._mapping_executor.shutdown(timeout_sec=1)
        if self._mapping_node:
            self._mapping_node.destroy_node()
        if ROS2_AVAILABLE and self._node:
            rclpy.try_shutdown()


cockpit_node = JetAutoCockpitNode()
saved_map_mgr = SavedMapManager(
    os.environ.get("JETAUTO_MAP_DIR", str(Path(WORKSPACE_DIR)/"src/slam/maps")),
    ROS_ENV_PREFIX, lambda: cockpit_node._node)


@asynccontextmanager
async def lifespan(app):
    cockpit_node.start()
    try:
        yield
    finally:
        cockpit_node.control.set_estop(True)
        try:
            await asyncio.to_thread(saved_map_mgr.stop)
        finally:
            cockpit_node.shutdown()


app.router.lifespan_context = lifespan


# ==================== HTTP / WebSocket 路由 ====================

@app.get("/api/health")
def get_health():
    return {
        "ok": True,
        "ros2_available": ROS2_AVAILABLE,
        "ros_initialized": cockpit_node._node is not None and cockpit_node.init_error is None,
        "ros_error": cockpit_node.init_error,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "control_ready": cockpit_node.control_ready(),
        "control_issues": cockpit_node.readiness_issues(),
        "devices": {"controller": Path("/dev/rrc").exists(), "lidar": Path("/dev/lidar").exists()},
        "control": {**cockpit_node.control.snapshot(),**cockpit_node.motion.snapshot()},
        "graph": cockpit_node.graph(),
        "capabilities": {"manual_control": True, "mapping_view": cockpit_node.mapping is not None,
                         "automatic_localization": cockpit_node.navigation is not None,
                         "navigation": cockpit_node.navigation is not None, "initialpose": cockpit_node.navigation is not None, "reset_odom": False},
        "mapping_error": cockpit_node.mapping_error,
        "navigation_error": cockpit_node.navigation_error,
        "saved_map": saved_map_mgr.status(),
        "estop_engaged": cockpit_node.estop_engaged,
        "hardware": collect_hardware_metrics(cockpit_node.battery_mv, cockpit_node.battery_ts),
        "nav_state": cockpit_node.navigation.status() if cockpit_node.navigation else cockpit_node.nav_state,
    }


@app.post("/api/cmd_vel")
def post_cmd_vel(cmd: TwistCommand):
    cockpit_node.send_cmd_vel(cmd)
    return {"ok": True, "topic": "/cmd_vel", "cmd": dict(cmd)}


@app.post("/api/estop")
def post_estop(engage: bool = True):
    cockpit_node.trigger_estop(engage)
    return {"ok": True, "estop_engaged": cockpit_node.estop_engaged, "scope": "gateway_manual_and_navigation"}


@app.get("/api/launches")
def get_launches():
    return {"modules": process_mgr.status_all()}


def costmap_settings_report(report):
    runtime = cockpit_node.costmap_parameters.snapshot() if cockpit_node.costmap_parameters else {}
    pending = differences(report['settings'], runtime)
    for name in ('global', 'local'):
        row = runtime.setdefault(name, {'available': False, 'issue': cockpit_node.costmap_parameters_error or '运行参数读取未就绪'})
        costs = getattr(cockpit_node.mapping, 'costmaps', None)
        grid, _ = costs.raw(name) if costs else (None, None)
        if grid:
            row['grid_resolution'] = grid['grid']['resolution']
            if abs(row['grid_resolution'] - report['settings'][name]['resolution']) > 1e-7:
                report['warnings'].append(name + ' 实际栅格分辨率与保存值不同；全局静态层会采用原始地图分辨率')
    return dict(report, ok=True, runtime=runtime, pending_changes=pending,
                restart_required=bool(pending) if pending is not None else None)


@app.get('/api/navigation/costmap-settings')
def get_costmap_settings():
    try:
        return costmap_settings_report(costmap_settings_store.read())
    except SettingsError as exc:
        raise HTTPException(exc.status, str(exc)) from exc


@app.post('/api/navigation/costmap-settings')
def save_costmap_settings(req: CostmapSettingsRequest):
    try:
        return costmap_settings_report(costmap_settings_store.save(req.settings, req.revision))
    except SettingsError as exc:
        raise HTTPException(exc.status, str(exc)) from exc


@app.post("/api/launches")
def control_launch(req: LaunchActionRequest):
    if req.action in ("start","restart") and req.module_id in ("slam","navigation") and saved_map_mgr.status()["running"]:
        raise HTTPException(409,"请先停止独立的保存地图服务，再启动 SLAM/导航")
    if req.action == "start":
        if req.module_id=="navigation" and not process_mgr.is_running('navigation'):
            if cockpit_node._node and cockpit_node._node.count_publishers('/map'):
                raise HTTPException(409,'已有地图发布者，请先停止保存地图服务或 SLAM')
            if cockpit_node.navigation:cockpit_node.navigation.invalidate()
        result=process_mgr.start(req.module_id, req.map_path)
        if req.module_id=='navigation' and cockpit_node.navigation and result['status']=='starting':
            cockpit_node.navigation.automatic.arm()
        return result
    if req.action in ("stop", "restart") and req.module_id in ("controller", "slam", "navigation"):
        if cockpit_node.navigation:cockpit_node.navigation.invalidate()
        cockpit_node.motion.end()
        cockpit_node.control.stop_manual()
    if req.action == "stop":
        result=process_mgr.stop(req.module_id)
        if req.module_id=='navigation' and cockpit_node.navigation:
            cockpit_node.navigation.invalidate(reset_unresolved=True)
        return result
    if req.action == "restart":
        process_mgr.stop(req.module_id)
        if req.module_id=='navigation' and cockpit_node.navigation:
            cockpit_node.navigation.invalidate(reset_unresolved=True)
        result=process_mgr.start(req.module_id, req.map_path)
        if req.module_id=='navigation' and cockpit_node.navigation:cockpit_node.navigation.automatic.arm()
        return result
    raise HTTPException(status_code=400, detail=f"不支持的动作: {req.action}")


@app.post("/api/map/save")
def save_map(req: MapSaveRequest):
    out_path = req.output_path.strip()
    if not out_path:
        raise HTTPException(status_code=400, detail="输出路径不能为空")
    allowed = Path(os.environ.get("JETAUTO_MAP_DIR", str(Path(WORKSPACE_DIR) / "src/slam/maps"))).resolve()
    resolved = Path(out_path).expanduser().resolve()
    if resolved == allowed or allowed not in resolved.parents:
        raise HTTPException(422, "地图只能保存在 " + str(allowed))
    if not cockpit_node._node or cockpit_node._node.count_publishers("/map") == 0:
        raise HTTPException(503, "当前没有 /map 发布者")
    out_path = str(resolved)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    cmd = (
        f"{ROS_ENV_PREFIX} && "
        f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(out_path)} "
        f"--ros-args -p map_subscribe_transient_local:=true"
    )
    try:
        res = subprocess.run(["/bin/bash", "-c", cmd], capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired as exc:
        raise HTTPException(504, "地图保存超时，请检查 /map 与 map_saver 日志") from exc
    return {
        "ok": res.returncode == 0,
        "returncode": res.returncode,
        "output_path": out_path,
        "stdout": res.stdout[-500:],
        "stderr": res.stderr[-500:],
    }


class SavedMapRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=100)


@app.get("/api/maps")
def list_saved_maps():
    return saved_map_mgr.catalog()


@app.post("/api/map/load")
def load_saved_map(req: SavedMapRequest):
    if process_mgr.is_running("slam") or process_mgr.is_running("navigation"):
        raise HTTPException(409,"请先停止 SLAM/导航组合模块，再独立加载保存地图")
    return saved_map_mgr.load(req.name)


def require_navigation():
    if cockpit_node.navigation is None:raise HTTPException(503,cockpit_node.navigation_error or "导航桥接尚未初始化")
    return cockpit_node.navigation


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw_deg: float
    confirmed: bool = False


@app.post("/api/localization/initialpose")
def set_initial_pose(req: InitialPoseRequest):
    return require_navigation().initial_pose(dict(req),req.confirmed)


@app.get("/api/navigation/status")
def navigation_status():
    return require_navigation().status()


@app.post("/api/localization/auto")
def start_auto_localization():
    return require_navigation().automatic.start()


@app.post("/api/localization/cancel")
def cancel_auto_localization():
    return require_navigation().cancel_automatic()


@app.post("/api/navigation/task")
def navigation_task(req: NavTaskRequest):
    return require_navigation().start(req.mode,[dict(w) for w in req.waypoints],req.loop_count)


@app.post("/api/navigation/precheck")
def navigation_precheck(req: NavTaskRequest):
    return require_navigation().precheck([dict(w) for w in req.waypoints])


@app.post("/api/navigation/cancel")
def navigation_cancel():
    return require_navigation().cancel('cancel')


@app.post("/api/navigation/pause")
def navigation_pause():
    return require_navigation().cancel('pause')


@app.post("/api/navigation/resume")
def navigation_resume():
    return require_navigation().resume()


@app.post("/api/map/unload")
def unload_saved_map():
    return saved_map_mgr.stop()


@app.get("/api/waypoints")
def list_waypoints():
    if WAYPOINTS_FILE.exists():
        return json.loads(WAYPOINTS_FILE.read_text(encoding="utf-8"))
    return {"waypoints": []}


@app.post("/api/waypoints")
def save_waypoints(waypoints: List[WaypointItem]):
    payload = {"waypoints": [dict(w) for w in waypoints]}
    filename = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=WAYPOINTS_FILE.parent,
                                         prefix='.waypoints-', delete=False) as output:
            filename = output.name
            json.dump(payload, output, ensure_ascii=False, allow_nan=False, indent=2)
        os.replace(filename, WAYPOINTS_FILE)
    except ValueError as exc:
        raise HTTPException(422, '站点参数必须是有限数字') from exc
    finally:
        if filename is not None:
            Path(filename).unlink(missing_ok=True)
    return payload


@app.websocket("/ws/telemetry")
async def websocket_telemetry(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            payload = await asyncio.to_thread(get_health)
            payload["launches"] = await asyncio.to_thread(process_mgr.status_all)
            await ws.send_json(payload)
            await asyncio.sleep(1)
    except (WebSocketDisconnect, RuntimeError, OSError):
        pass


@app.websocket("/ws/mapping")
async def websocket_mapping(ws: WebSocket):
    """Read-only channel, independent of control/telemetry reconnects."""
    await ws.accept()
    revision = None
    costmap_revisions={}
    try:
        while True:
            if cockpit_node.mapping is None:
                payload = {"type": "mapping", "available": False,
                           "detail": cockpit_node.mapping_error or "ROS 地图订阅尚未就绪"}
            else:
                payload = await asyncio.to_thread(cockpit_node.mapping.snapshot, revision, costmap_revisions)
                revision = payload["map_revision"]
                for name,entry in payload.get('costmaps',{}).items():
                    if entry.get('grid'):costmap_revisions[name]=entry['revision']
                    if not entry.get('available'):costmap_revisions.pop(name,None)
            await ws.send_json(payload)
            await asyncio.sleep(0.1 if payload["available"] else 1)
    except (WebSocketDisconnect, RuntimeError, OSError):
        pass


@app.websocket("/ws/control")
async def websocket_control(ws: WebSocket):
    await ws.accept()
    owner = "ws:" + uuid.uuid4().hex
    try:
        await ws.send_json({"type": "ready", "timeout_sec": 0.5})
        while True:
            data = await ws.receive_json()
            try:
                if not isinstance(data, dict):
                    raise ValueError("控制消息必须是 JSON 对象")
                cmd = TwistCommand(**dict(data, client_id=owner))
                cockpit_node.send_cmd_vel(cmd, owner)
                await ws.send_json({"ok": True})
            except HTTPException as exc:
                await ws.send_json({"ok": False, "status": exc.status_code, "detail": exc.detail})
            except (ValueError, TypeError) as exc:
                cockpit_node.control.disconnect(owner)
                await ws.send_json({"ok": False, "status": 422, "detail": str(exc)})
    except (WebSocketDisconnect, RuntimeError, OSError, ValueError):
        pass
    finally:
        cockpit_node.control.disconnect(owner)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("JETAUTO_AGENT_PORT", "8000")))
