# JetAuto Pro Web 驾驶舱 v1.8.0

适用平台：Jetson Orin NX Super、Ubuntu 22.04、ROS 2 Humble。默认机器人地址 `192.168.3.73`；从机器人网页打开时自动使用页面所在主机。

本版新增启动自动定位：整图激光匹配，必要时低速原地旋转融合周围扫描，AMCL 验证通过后开放导航，无需人工标注初始位姿。失败会保持停车并显示原因。启动前需留出整车及机械臂旋转空间。操作、配置和边界见 [AUTO_LOCALIZATION_V1.8.md](AUTO_LOCALIZATION_V1.8.md)。已有导航 API 说明见 [NAVIGATION_V1.7.md](NAVIGATION_V1.7.md)。

## 已实现与范围

- 默认实车模式：实际连接 rosbridge、网关遥测及手动控制 WebSocket；接收硬件数据、电池样本与 `/odom`。缺失或过期数据显示未知。
- 手动控制统一经过网关：限幅 X/Y ±0.20 m/s、Yaw ±0.50 rad/s；一次只允许一个客户端持有控制租约。浏览器以 100ms 心跳续约，网关在 500ms 无指令后发布零速；连接关闭也撤销租约。
- 模块启停调用实际 HTTP API，展示 PID、退出状态与日志；检测到外部进程时仅展示，不接管或停止它。进程存活与 ROS 就绪状态分别判断。
- RGB 使用 8080 MJPEG。视频失败显示错误并重试，不用演示画面替代。原始 16UC1 深度入口禁用，需要另行实现深度可视化。
- 网页 2D 地图显示真实 `/map`、TF 对齐的激光与机器人位置，支持平移、缩放、自动适应范围和图层开关；网关使用独立只读节点及执行器订阅地图、激光和 TF。
- 已保存地图通过“加载地图（仅显示）”独立加载、激活和切换，不启动运动驱动；需要先停止其他地图发布源。仅加载不提供 AMCL 定位或导航。
- 地图保存调用真实 `map_saver_cli`，限制输出到指定地图目录。
- 演示模式保留模拟数据，切换模式前撤销手动控制。

Nav2 使用实际 ROS 2 Action，状态来自反馈与结果；未定位、TF 缺失、生命周期未激活、低电压或急停时禁止驱动。单点使用 NavigateToPose，多点连续经过使用 NavigateThroughPoses，逐点巡航由网关串行调用 NavigateToPose，支持每点驻留和循环。里程计重置、实车快照、3D 与代价地图图层仍未接入。

网关软急停覆盖本网关手动与导航速度，并取消本网关发起的任务。它不是硬件急停，无法阻断绕过网关的外部 ROS 发布者；检测到其他 `/cmd_vel` 发布源会禁止网关导航。解除急停不会恢复历史速度或自动恢复任务。

## 文件

```text
frontend/jetauto_pro_cockpit.html  页面
frontend/cockpit_transport.js    实车连接、重连、心跳与 HTTP 请求
frontend/live_map.js             实时地图、激光、位姿渲染与地图只读连接
backend/agent_server.py          FastAPI 与 ROS 节点
backend/control_safety.py        手动租约、导航速度仲裁和超时停车逻辑
backend/navigation_bridge.py     初始定位、Action 任务编排和就绪检查
backend/automatic_localization.py 自动定位流程及受限原地旋转
backend/global_scan_matcher.py    整图扫描匹配与相似候选检查
backend/cockpit_navigation.launch.py  完整导航组合启动
backend/config/                 本平台专用 Nav2 与 DWB 参数
backend/mapping_stream.py        地图/激光订阅、TF 缓冲与时间对齐
backend/saved_maps.py            地图目录与独立地图进程管理
backend/saved_map_worker.py      启动并激活唯一命名的 map_server
backend/ros_env.sh               统一 ROS/厂商/Orbbec 工作空间环境
backend/start_cockpit_agent.sh   启动 rosbridge 和网关
backend/requirements.txt         非 ROS 的 Python 依赖
tests/                          无实车控制与接口测试、浏览器模拟服务测试
```

前端需要同时部署 HTML 和两个 JS 文件，后端也需部署整个目录。页面样式和演示 3D 仍使用外部 CDN；实车连接代码本地加载。

## 部署与启动

2026-11-15 已在用户授权下备份并更新机器人 `/home/ubuntu/jetauto_cockpit/`，安装导航依赖并重启网关。下列命令用于以后手动启动；不要重复运行两个网关。重新部署须同时覆盖整个 `frontend/`、`backend/`（包括 config），并先备份、停止旧网关。

```bash
cd /home/ubuntu/jetauto_cockpit
python3 -m pip install -r backend/requirements.txt
ROS_DOMAIN_ID=3 bash backend/start_cockpit_agent.sh
```

`ros_env.sh` 依次加载 ROS、厂商 `~/ros2_ws/.typerc`、third_party、Orbbec 和 ros2_ws 的 Bash 环境；不会在 Bash 中加载包含 Zsh 配置的 `.robotrc`。Orbbec 默认目录是 `~/third_party/orbbec_ws`，不同安装路径可通过 `ORBBEC_WS` 指定。显式传入的 `ROS_DOMAIN_ID` 会保留，未指定时默认 3，以匹配本次检查的车端网关。9090 已存在时脚本复用该监听服务，需确保它也属于 Domain 3。

如果 3000 静态服务尚未运行，在另一终端执行（已有服务无需重复启动）：

```bash
cd /home/ubuntu/jetauto_cockpit
python3 -m http.server 3000 --bind 0.0.0.0 --directory frontend
```

打开 `http://192.168.3.73:3000/jetauto_pro_cockpit.html`，强制刷新清除旧缓存。页面自动尝试连接，但不会自动启动运动驱动。

1. 查看 API 在线和真实模块状态。
2. 依次启动“底盘控制与 EKF”“深度相机”“Web 视频流服务”；需要雷达再启动 lidar。
3. 控制就绪要求网关初始化成功、`/dev/rrc` 存在、`/cmd_vel` 有订阅者，底层板有电池发布者，且收到 5 秒内、电压至少 10.6V 的样本；等待相机和视频进程实际就绪。视频失败会每 10 秒重试，也可手动重新加载。
4. 如状态显示“外部进程”，在其原启动终端检查；网关不会强行关闭它。

完整底盘必须使用 `ros2 launch controller controller.launch.py`，不要同时单独启动 `ros_robot_controller`。SLAM/导航组合 launch 自带基础驱动（导航请直接启动组合模块，勿先单独启动这些基础模块），因此与单独运行的 controller、depth_camera、lidar 互斥；切换前需先手动停止相应模块。停止受管底盘/SLAM/导航时撤销手动速度并发送零速，不会自动加锁；用户主动触发的急停仍需手动解除。

RGB 独立验证地址：

```text
http://192.168.3.73:8080/stream?topic=/depth_cam/rgb/image_raw&type=mjpeg&quality=80
```

## API

自动定位新增接口见 AUTO_LOCALIZATION_V1.8.md。

| 接口 | 用途 |
| --- | --- |
| `GET /api/health` | ROS 初始化、错误、Domain、设备状态、control_issues 原因、订阅/发布数量、功能能力、mapping_error 与硬件状态 |
| `GET /api/launches` | 模块进程状态、是否外部进程、日志 |
| `POST /api/launches` | `{module_id, action: start/stop/restart, map_path?}`；navigation 的 map_path 是地图名，如 `map_01` |
| `POST /api/estop?engage=true/false` | 锁定/解除网关手动与导航急停 |
| `POST /api/cmd_vel` | `{client_id, linear_x, linear_y, angular_z}`；HTTP 客户端须提供自己的唯一 ID 并持续续约 |
| `WS /ws/control` | 页面使用的有序控制通道；消息 `{linear_x, linear_y, angular_z}`；服务器回复成功或错误，断开撤销自身租约 |
| `WS /ws/mapping` | 只读地图显示通道；地图按修订更新，激光与位姿约 10Hz，提供 TF 与数据状态 |
| `WS /ws/telemetry` | 每秒发送 health 字段及 launches 列表 |
| `GET /api/maps` | 列出可用保存地图名与独立服务状态 |
| `POST /api/map/load` | `{name: "map_501"}`，仅加载并激活保存地图服务 |
| `POST /api/map/unload` | 停止受管的保存地图服务 |
| `POST /api/map/save` | `{output_path}`，仅允许在 `JETAUTO_MAP_DIR` 内保存；默认 `~/ros2_ws/src/slam/maps` |
| `POST /api/localization/initialpose` | `{x,y,yaw_deg,confirmed:true}`；明确确认实际位姿后提交 |
| `GET /api/navigation/status` | 就绪原因、生命周期、任务反馈和结果 |
| `POST /api/navigation/task` | `{mode,waypoints,loop_count}`，坐标为 map 米制；详见 NAVIGATION_V1.7.md |
| `POST /api/navigation/pause` | 取消当前 Action 并记录进度，等反馈进入 PAUSED |
| `POST /api/navigation/resume` | 重新下发未完成目标；只允许 PAUSED 且定位就绪 |
| `POST /api/navigation/cancel` | 关闭速度授权并取消当前任务 |
| `GET/POST /api/waypoints` | 保留后端站点存储接口；当前 UI 站点编辑为本地草稿/JSON 导出，不自动持久化 |

地图保存要求 `/map` 发布者存在。导航 launch 接受地图名，不接受任意 YAML 路径。健康检查的 `ok=true` 表示网关存活，不能替代 `ros_initialized`、`control_ready` 和实际传感器样本检查。

## 本地验证

测试使用假发布者及模拟 HTTP/WebSocket，不会连接机器人：

```bash
python3 -m pip install -r tests/requirements.txt
python3 -m unittest discover -s tests -v
python3 -m playwright install chromium
python3 tests/browser_smoke.py
python3 tests/browser_mapping.py
python3 tests/browser_navigation.py
node tests/test_map_math.js
node --check frontend/cockpit_transport.js
bash -n backend/start_cockpit_agent.sh
bash -n backend/ros_env.sh
```

浏览器测试也可设置 `COCKPIT_BROWSER_EXE` 使用现有 Chrome/Edge。此前 v1.7 通过 40 项 Python 测试、地图几何测试和三组 headless Edge 页面测试；真实 Nav2 的隔离域测试通过，已部署到机器人。实际页面已确认 map_501 与 RGB 视频可用，地图为 119 × 129；实车位置未知且电压偏低，未进行实际行驶验收。详见 NAVIGATION_V1.7.md 和 DEPLOY_CHECK_2026-11-15.md。
