# JetAuto Pro Web 驾驶舱 1.0

发布版本：**1.0.0**。整理日期：2026-11-21。当前功能以本 README 和源码为准，原有分散的部署、导航、自动定位及检查文档已合并。

本项目提供机器人网页控制、遥测、视频、建图观察、保存地图管理、自动定位与 Nav2 导航。网页和网关调用机器人原有 ROS 2 底盘控制栈与 Nav2；没有重写麦轮运动学、电机 PID 或底层固件。

## 目录

- [平台与访问地址](#平台与访问地址)
- [功能与限制](#功能与限制)
- [代码结构与控制链](#代码结构与控制链)
- [安装部署与启动](#安装部署与启动)
- [日常操作](#日常操作)
- [自动定位](#自动定位)
- [导航参数与控制保护](#导航参数与控制保护)
- [HTTP 与 WebSocket API](#http-与-websocket-api)
- [配置与环境变量](#配置与环境变量)
- [常见问题](#常见问题)
- [测试与实车验证记录](#测试与实车验证记录)
- [备份与回退](#备份与回退)

## 平台与访问地址

| 项目 | 配置 |
| --- | --- |
| 机器人 | 幻尔 JetAuto Pro，麦轮底盘 |
| 主控 | Jetson Orin NX Super |
| 系统 | Ubuntu 22.04、ROS 2 Humble |
| 激光雷达 | YDLIDAR G4 |
| 深度相机 | Orbbec Astra Pro Plus |
| 当前默认 IP | `192.168.3.73`，网页优先采用页面所在主机 |
| ROS Domain | 默认 `3`，网关、rosbridge 和各 ROS 节点必须一致 |
| 车端项目目录 | `/home/ubuntu/jetauto_cockpit` |
| 默认地图 | `map_501` |
| 默认地图目录 | `/home/ubuntu/ros2_ws/src/slam/maps` |

| 服务 | 地址与作用 |
| --- | --- |
| 网页 3000 | `http://192.168.3.73:3000/jetauto_pro_cockpit.html` |
| 网关 8000 | `http://192.168.3.73:8000`，HTTP API 与 WebSocket |
| 接口调试 | `http://192.168.3.73:8000/docs`；版本信息见 `/openapi.json` |
| rosbridge 9090 | `ws://192.168.3.73:9090` |
| 视频 8080 | RGB MJPEG 服务，需要单独启动视频模块 |

当前没有接口认证或 TLS，应在可信局域网使用。页面样式与演示 3D 仍依赖外部 CDN，通信代码在项目内。

## 功能与限制

| 功能 | 当前状态 |
| --- | --- |
| 实车连接与遥测 | 默认实车模式，真实 HTTP/WebSocket、ROS 图、电池与硬件指标；缺失或过期数据明确显示未知 |
| 手动驾驶 | 网关单客户端控制租约、心跳、限速、松手/失焦/断线停车 |
| 模块启停 | 实际启动/停止受管 ROS 进程，显示 PID、日志及外部进程；组合模块与基础驱动互斥 |
| RGB 视频 | 8080 MJPEG；失败显示错误并重试，不用演示画面替代 |
| 2D 地图 | `/map`、按扫描时间对齐的激光、机器人位姿和 `/plan`；支持平移、缩放、适应地图、居中和图层切换 |
| SLAM 建图 | 启停已有 SLAM 组合模块，网页观察真实建图，手动驾驶扩展地图并保存 |
| 保存地图管理 | 列表、独立加载、激活、切换、卸载与保存 |
| 启动自动定位 | 整图激光匹配；必要时低速原地旋转融合扫描；AMCL 验证后开放导航，无需人工标注 |
| Nav2 任务 | 单点、多点连续经过、逐点驻留与循环、真实反馈、暂停/恢复/取消 |
| 演示模式 | 独立模拟数据；与实车模式分开 |
| 尚未接入 | 原始深度图可视化、深度点云避障、实车 3D、代价地图图层、里程计重置、实车快照 |

自动定位可能因对称场景、重复走廊、遮挡或地图变化而失败；系统保持停车，不保证任意环境都能定位成功。当前 footprint 为待实测参数，机械臂伸出后的碰撞外廓未验证。软件测试通过不代表完成实车行驶验收，详见[验证记录](#测试与实车验证记录)。

## 代码结构与控制链

```text
frontend/
  jetauto_pro_cockpit.html         网页入口与操作界面
  cockpit_transport.js            连接、重连、心跳与 HTTP 请求
  live_map.js                     地图、激光、位姿与路径渲染
backend/
  agent_server.py                 FastAPI、遥测、ROS 节点与模块管理
  control_safety.py               手动租约、导航/定位速度仲裁与超时停车
  navigation_bridge.py            Action 任务编排、初始定位与就绪检查
  automatic_localization.py       自动定位、受限原地旋转与验证流程
  global_scan_matcher.py          全局扫描匹配、候选分差与局部峰形检查
  mapping_stream.py               地图/激光订阅及时间对齐 TF
  saved_maps.py                   地图目录与独立地图进程管理
  saved_map_worker.py             启动并激活独立 map_server
  cockpit_navigation.launch.py    完整导航组合启动
  config/nav2_params.yaml         AMCL、规划、代价地图、行为与速度平滑
  config/nav2_controller.yaml     DWB 全向控制器与速度限制
  ros_env.sh                     统一 Bash ROS 环境
  start_cockpit_agent.sh          启动 rosbridge 和网关
  requirements.txt               Python 依赖
tests/                           控制、API、定位、地图及浏览器测试
README.md                        唯一项目说明
```

手动驾驶：网页 → 网关租约与保护 → `/cmd_vel` → 原有 `controller` 控制栈 → `ros_robot_controller` → 控制板和电机。

自主导航：网页任务 → 网关 Action 客户端 → Nav2 → `/cockpit/nav_raw` → `velocity_smoother` → `/cockpit/nav_cmd_vel` → 网关仲裁/限速 → `/cmd_vel` → 原有底盘控制栈。

定位旋转使用网关独立授权，仅输出零平移及受限角速度。它不依赖尚未完成的 map 定位，也不会接受私有导航速度覆盖定位指令。

## 安装部署与启动

### 依赖

适用于已有厂商 ROS 工作空间和设备规则的机器人。安装 ROS 依赖前先加载正确的软件源：

```bash
sudo apt-get install -y --no-remove \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-theta-star-planner \
  ros-humble-rosbridge-server python3-numpy python3-scipy

cd /home/ubuntu/jetauto_cockpit
python3 -m pip install -r backend/requirements.txt
```

还需要厂商 `controller`、`ros_robot_controller`、`peripherals`、`slam`、机器人模型、雷达与 Orbbec 驱动，以及可运行的 `web_video_server`。Nav2 启动前会检查关键包和设备；仅能导入 ROS 模块不代表驱动齐全。

### 更新已有部署

1. 在网页取消导航/自动定位、停车，停止受管组合模块与视频服务，再在原网关终端 Ctrl+C 退出网关，避免子进程失去管理。
2. 备份 `/home/ubuntu/jetauto_cockpit`。将本项目完整的 `frontend/`、`backend/` 和 `README.md` 复制到车端对应位置；包括两个 JS、所有 Python、Shell 和 `config/`。保留自己的 `waypoints_store.json` 和配置。
3. 无需覆盖厂商 `~/ros2_ws`、地图或重新编译厂商包。启动新网关，浏览器 Ctrl+F5 强制刷新。不要同时运行两个网关。

历史文档清理和 1.0.0 版本归档在本地项目完成；车端正在运行的进程版本以 `/openapi.json` 为准。版本归档本身不重启机器人服务。

### 启动服务

终端 1：

```bash
cd /home/ubuntu/jetauto_cockpit
ROS_DOMAIN_ID=3 bash backend/start_cockpit_agent.sh
```

终端 2（3000 未运行时）：

```bash
cd /home/ubuntu/jetauto_cockpit
python3 -m http.server 3000 --bind 0.0.0.0 --directory frontend
```

启动脚本只提供网关和按需启动 rosbridge，不会自动启动硬件驱动。硬件模块从网页启动。当前没有配置开机自启；重启机器人后仍需启动这些服务。

`ros_env.sh` 加载 ROS Humble、厂商 `.typerc` 和 third_party/Orbbec/ros2_ws 的 Bash overlay；厂商 `.robotrc` 含 Zsh 配置，不能直接交给 Bash。网关管理的每个子进程都会加载统一环境，不需要逐一人工 source。

需要另开 Bash 终端使用 ROS CLI 时，先执行：

```bash
source /home/ubuntu/jetauto_cockpit/backend/ros_env.sh
export ROS_DOMAIN_ID=3
```

## 日常操作

### 手动驾驶与视频

在实车模式启动“底盘运动与 EKF 控制栈”，需要画面时再启动“深度相机”和“Web 视频流服务”。雷达按需启动。等待底盘控制链、电池和实际传感器数据就绪后操作。

完整底盘使用 `ros2 launch controller controller.launch.py`。单独的 `ros_robot_controller` 是底层板驱动，不能代替 Twist 转换、里程计、EKF 和模型 TF；也不要同时重复启动两套底盘。

RGB 独立检查地址：

```text
http://192.168.3.73:8080/stream?topic=/depth_cam/rgb/image_raw&type=mjpeg&quality=80
```

视频 topic 参数需要保留 `/`，本机旧版视频服务不解码 `%2F`。原始 `16UC1` 深度不能直接按 `bgr8` 彩色图转 MJPEG，本版未实现深度可视化。

### 手动驾驶建图

1. 停止导航、独立保存地图服务和单独运行的底盘/相机/雷达模块。
2. 启动“2D SLAM 在线建图”组合模块；它使用已有 `ros2 launch slam slam.launch.py`，自带基础驱动。
3. 等待真实 `/map`、`/scan`、里程计和 TF，使用网页手动驾驶逐步扩展地图。
4. 地图白色为空闲、黑色为障碍、灰色为未知；激光红点按扫描时间变换，车体箭头显示地图位置和朝向。TF 缺失或数据过期时不叠加失效位姿/激光。
5. 在默认地图目录下指定保存路径，例如 `/home/ubuntu/ros2_ws/src/slam/maps/map_502`，不带扩展名，再保存地图。只有实际保存成功才认为生成文件。

地图拖拽平移、滚轮缩放；“适应地图”显示全图，“居中车体”对准有效机器人位置。地图断开时保留最后底图并标为旧数据，重连后恢复。

### 仅查看保存地图

停止 SLAM/导航和其他 `/map` 发布源，地图名填 `map_501`，点击“加载地图（仅显示）”。后端只启动唯一命名的独立 map_server，并完成 configure → activate；不启动底盘、雷达、相机、AMCL 或导航任务。

单独看底图不要求硬件节点运行，也不表示机器人已定位。结束时停止保存地图服务。进入导航前先卸载该独立地图服务，避免重复 `/map` 发布者。

### 导航

1. 确认地图对应当前环境、底盘带载电压正常，整车和机械臂周围留出旋转空间。
2. 停止 SLAM、独立保存地图服务和单独底盘/相机/雷达模块，直接启动“Nav2 自主定位与导航”。组合模块包含这些基础驱动，不需要逐个再启动；视频服务可独立运行。
3. 等待自动定位成功，且页面显示定位、TF 与导航节点已就绪。
4. “指哪走哪”在地图可通行区域按下并拖出目标朝向，松开即提交单点任务。也可以先拾取/添加站点，选择模式和循环次数后启动任务。
5. 暂停会取消底层 Action 并等待终态，显示 PAUSED 后才能恢复。恢复重发未完成目标并保留剩余驻留时间；取消不自动恢复。

组合模块包含的子节点可能显示为外部进程，应停止对应的受管组合模块。真正由其他终端启动的外部进程，需在其原终端停止；网关不会接管它。

## 自动定位

默认启动导航后自动定位，无需人点击地图。流程为：等待数据 → 整图搜索 → 必要时原地旋转 → 融合搜索 → AMCL 验证 → 成功或保持停车。

- 使用 `/scan` 和完整 `base_footprint ← lidar_frame` TF，搜索整张地图的 X/Y 与 360° 朝向。粗搜索约 0.20m/10°，精搜索约 0.05m/2°。
- 检查有效回波、端点命中、射线自由空间和相似候选分差。有效射线至少 60 条、角度覆盖至少 120°；地图上限 100 万格，粗搜索位置上限 12000 个。
- 静止匹配不足且允许旋转时，以角速度不超过 0.10rad/s 原地转约一圈，按 odom 融合不同朝向扫描，不发送平移速度。
- 匹配 score 至少 0.68、端点命中比例至少 0.65、自由空间比例至少 0.95、候选分差至少 0.06，并通过局部峰形检查后，才自动发布 `/initialpose`。这些指标不是定位正确概率。
- 自动初始化协方差根据局部峰形设置，位置方差下限 0.0125m²、偏航方差下限为 5° 的方差。随后通过 `/request_nomotion_update` 请求 AMCL 更新，检查新响应、协方差、扫描一致性和 TF。
- 验证至少 8 个有效新样本、稳定 3 秒；通过后标为 `initial_pose_source=automatic`。候选已提交但未验证时为 `auto_pending`，仍禁止导航。
- 旋转最多 85 秒；允许旋转时总定位上限 150 秒，仅静止模式上限 60 秒。失败后保持停车，不会自动反复旋转。

定位旋转遇到急停、低电压、驱动/激光/TF 中断、其他 `/cmd_vel` 发布者、可见旋转包络 0.43m 内障碍、异常平移超过 0.10m 或朝向跳变时停止。指令或条件超过 0.3 秒未更新即输出零速。

0.43m 包络基于当前车体估计和可见激光回波，不能覆盖盲区、机械臂伸出或高于雷达平面的障碍。完全相似场景可能无法唯一定位。启动前留出整车及机械臂旋转空间；匹配验证是保守启发式，不能保证识别所有错误定位。

网页提供“重新自动定位”和“取消自动定位”。手动标注仅作备用：先取消自动定位，在地图按住并拖出实际位置/车头方向，再明确确认。橙色草稿不会自动发布，不能把地图中心当作实际位置。

导航过程中 AMCL 持续更新定位。本版自动全局定位针对启动和显式重试；没有实现定位丢失后自动重定位并恢复原任务的完整流程。

## 导航参数与控制保护

配置在项目 `backend/config/` 和专用 launch 中，保留厂商 navigation 文件。

| 参数 | 当前值 |
| --- | --- |
| 时钟 | `use_sim_time=false` |
| AMCL 运动模型 | `nav2_amcl::OmniMotionModel` |
| 初始位姿 | `set_initial_pose=false`；由自动定位或可选人工确认发布，不默认设为原点 |
| TF 主链 | `map → odom → base_footprint → base_link / lidar_frame` |
| 局部 / 全局代价地图 | `odom` / `map`，车体 frame 为 `base_footprint` |
| 障碍信息 | `/scan`；局部障碍层/膨胀层，全局增加静态地图层 |
| 全局规划 / 局部控制 | ThetaStar / DWB 全向控制 |
| 手动速度上限 | X/Y ±0.20m/s，Yaw ±0.50rad/s |
| 导航速度上限 | X -0.10～+0.15m/s，Y ±0.10m/s，Yaw ±0.40rad/s |
| 定位旋转上限 | X/Y 为零，Yaw ±0.10rad/s |
| 到点容差 | 位置 0.15m，偏航 0.20rad |
| footprint | 0.50m × 0.44m 矩形，padding 0.02m；仍待实测 |
| 底盘电池条件 | 样本 5 秒内，电压至少 10.6V |
| 手动/导航速度超时 | 0.5 秒 |

底盘就绪还要求网关初始化成功、`/dev/rrc` 存在、`/cmd_vel` 有实际订阅者及底层板电池发布者。`ok=true` 只表示网关存活；不能替代 `control_ready`、传感器样本和导航 readiness。

Nav2 接受本网关任务后才开放导航速度；无任务时忽略私有导航速度。导航执行或自动定位期间非零手动请求返回 409。导航中的零速请求会取消导航；取消任务、取消自动定位和软急停会撤销对应速度授权。恢复或解除急停不会自动恢复旧速度。

定位、TF 或就绪条件丢失会阻断速度，持续丢失会取消导航。接收/取消确认超时会保留未确认状态，需成功停止或重启导航后再试。模块停止会撤销控制并发送零速，不会无故新建急停锁。

软急停只覆盖本网关路径和本网关发起的任务，不能替代硬件急停，也不能阻止外部程序绕过网关直接驱动。深度点云尚未接入避障层，不能依靠本版避让雷达平面之外的障碍。

## HTTP 与 WebSocket API

基础地址为 `http://<机器人IP>:8000`。坐标单位米，地图参考系 `map`；`yaw_deg` 为逆时针角度。以下坐标仅为字段示例，不是机器人实际目标。

### HTTP 接口

| 接口 | 请求与用途 |
| --- | --- |
| `GET /api/health` | ROS 初始化、Domain、设备、图、控制来源/原因、硬件样本、capabilities、saved_map 和 nav_state |
| `GET /api/launches` | 模块进程、PID、managed/external、日志和启动参数 |
| `POST /api/launches` | `{module_id,action,map_path?}`，action 为 start/stop/restart |
| `POST /api/estop?engage=true/false` | 锁定/解除网关软急停 |
| `POST /api/cmd_vel` | `{client_id,linear_x,linear_y,angular_z}`；HTTP 客户端使用唯一 ID 并持续续约 |
| `GET /api/maps` | `{maps,directory,saved_map}`；列出允许加载的 YAML 地图名 |
| `POST /api/map/load` | `{"name":"map_501"}`，独立启动并激活地图服务，仅显示 |
| `POST /api/map/unload` | `{}`，停止受管独立地图服务 |
| `POST /api/map/save` | `{output_path}`，map_saver_cli 输出路径，不含扩展名 |
| `POST /api/localization/auto` | `{}`，启动/重试自动定位；活动导航或手动控制存在时拒绝 |
| `POST /api/localization/cancel` | `{}`，取消自动定位并撤销旋转授权 |
| `POST /api/localization/initialpose` | `{x,y,yaw_deg,confirmed:true}`，备用人工初始位姿 |
| `GET /api/navigation/status` | 任务、readiness、localization、生命周期、反馈与 ROS 结果 |
| `POST /api/navigation/task` | `{mode,waypoints,loop_count}`，提交导航任务 |
| `POST /api/navigation/pause` | `{}`，关闭速度授权、取消 Action，等终态进入 PAUSED |
| `POST /api/navigation/resume` | `{}`，PAUSED 且就绪时重发未完成目标 |
| `POST /api/navigation/cancel` | `{}`，关闭授权并请求取消任务 |
| `GET /api/waypoints` | 读取后端站点存储，返回 `{waypoints:[...]}` |
| `POST /api/waypoints` | 请求体为 WaypointItem 数组，保存到 `backend/waypoints_store.json` |

模块 ID：`controller`、`depth_camera`、`lidar`、`slam`、`navigation`、`web_video_server`。

启动导航示例：

```json
{"module_id":"navigation","action":"start","map_path":"map_501"}
```

`map_path` 为地图名，不含目录和 `.yaml`；网关解析到允许地图目录内的完整 YAML 路径，交给本项目 launch。地图名仅允许字母、数字、下划线和连字符。独立地图服务只操作自己创建的节点，拒绝冲突的外部地图源。

保存地图示例：

```json
{"output_path":"/home/ubuntu/ros2_ws/src/slam/maps/map_502"}
```

必须存在实际 `/map` 发布者，路径必须在 `JETAUTO_MAP_DIR` 内。检查响应 `ok`、returncode 和错误输出。

### 导航任务

```json
{
  "mode":"waypoints",
  "loop_count":1,
  "waypoints":[
    {"id":"p1","name":"目标1","x":1.2,"y":-0.4,"yaw_deg":90,"dwell_sec":2}
  ]
}
```

| mode | 执行方式 |
| --- | --- |
| `single` | 恰好一个点，原生 `/navigate_to_pose`，不执行逐点驻留 |
| `through_poses` | 原生 `/navigate_through_poses` 连续经过点列，不执行每点 dwell |
| `waypoints` | 网关逐点调用 `/navigate_to_pose`，成功后驻留，支持循环 |

逐点模式未调用原生 `/follow_waypoints`，用于支持不同驻留时间及恢复进度。点数 1～100，循环次数 1～10，dwell_sec 0～300 秒。坐标必须有限，目标须位于地图已知、占用值小于 65 的栅格；该检查不替代路径可达性或足迹碰撞判断。

提交成功返回 `{ok:true,task_id,state:"pending"}`，只表示已提交；真实接收、执行和结果以反馈为准。任务上下文在内存中，网关重启不恢复旧任务，需要重新完成定位。

`status_text`：IDLE、PENDING、EXECUTING、DWELLING、CANCELING、PAUSED、CANCELED、SUCCEEDED、ABORTED。PAUSED 与 CANCELED 的 status_code 都为 5，需同时读取 paused/status_text。`ros_result_status` 为底层结果：4 成功、5 取消、6 中止。

反馈含 distance_remaining（米）、estimated_time_remaining/navigation_time（秒）、number_of_recoveries、从 0 开始的 current_waypoint_index、从 1 开始的 current_loop、dwell_remaining 和 waypoints。ETA 为 0 可能表示暂无估计；恢复次数与耗时以当前底层 Action 为单位。

网页站点编辑为本地草稿与 JSON 导出，不自动持久化到站点 API。暂停实现为取消并重发，不是冻结 Nav2 内部状态。

### 定位状态与备用手动位姿

`localization` 阶段：IDLE、WAITING、MATCHING、ROTATING、VERIFYING、SUCCEEDED、FAILED、CANCELED；包含 message/error、rotation_allowed、rotation_degrees、elapsed_sec 与 match。`control.source=localization` 表示定位旋转占用控制权。

`readiness.initial_pose_source` 为 automatic、auto_pending 或 manual；`initial_pose_confirmed` 同时表示自动验证通过或人工确认，不要求人点击。

备用初始位姿示例：

```json
{"x":1.2,"y":-0.4,"yaw_deg":90,"confirmed":true}
```

AMCL 必须 active，点需可通行且无自动定位、活动导航或手动租约。人工初始化位置方差为 0.25m²，偏航为 15° 的方差，返回 waiting_for_amcl，不能立即视为定位成功。

### WebSocket 与错误

| 通道 | 内容 |
| --- | --- |
| `WS /ws/control` | 有序手动控制消息 `{linear_x,linear_y,angular_z}`；服务端回复 ready/成功/错误，断开撤销自己的租约 |
| `WS /ws/telemetry` | 每秒推送 health 字段及 launches 列表 |
| `WS /ws/mapping` | 只读地图通道，修订变化时推送 map，激光/位姿约 10Hz，附 TF 和数据状态 |

地图结构含 width、height、resolution、origin、data、frame_id；扫描点已变换到地图坐标。`/plan` 仅发送 frame 匹配、最近 3 秒内的折线。图层未接入时保持禁用。

常见 HTTP 错误：409 控制/模块/地图冲突或未就绪，423 急停，422 参数或位置无效，404 地图不存在，503 驱动/包/服务不可用，504 等待超时。ROS Action 字段以机器上的 Humble 接口和 [Nav2 Humble 文档](https://api.nav2.org/humble/actions/navigation/)为准。

## 配置与环境变量

| 环境变量 | 默认值/用途 |
| --- | --- |
| `ROS_DOMAIN_ID` | `3`，统一所有 ROS 进程 |
| `JETAUTO_WS` | `~/ros2_ws` |
| `THIRD_PARTY_WS` | `~/third_party/third_party_ws` |
| `ORBBEC_WS` | `~/third_party/orbbec_ws` |
| `JETAUTO_MAP_DIR` | `$JETAUTO_WS/src/slam/maps` |
| `JETAUTO_AGENT_PORT` | `8000` |
| `JETAUTO_AUTO_LOCALIZATION` | 默认 true；false 关闭自动触发，仍可通过 API 主动请求 |
| `JETAUTO_AUTO_ROTATE` | 标准启动脚本默认 true；直接运行 Python 未设置时默认 false |
| `COCKPIT_BROWSER_EXE` | 测试时指定已有 Chrome/Edge 可执行文件 |

强制仅静止定位可这样启动：

```bash
ROS_DOMAIN_ID=3 JETAUTO_AUTO_ROTATE=false bash backend/start_cockpit_agent.sh
```

启动脚本会复用已有 9090 监听者，需自行确认其 Domain；不会替换外部 rosbridge。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| 网页在线但无法控制 | 查看 control_issues、`/dev/rrc`、实际底盘订阅者、电池新鲜度和急停；仅有话题名称或 odom 节点不代表底层板工作 |
| 视频黑屏/8080 拒绝连接 | 启动相机和视频模块，查看实际 RGB 发布者；先单独打开 RGB URL |
| `16UC1` 转 `bgr8` 报错 | 原始深度不是彩色图，本版禁用该视频入口 |
| `bad substitution`、`cd: -q` | Bash 中加载了 Zsh 配置；使用项目 ros_env.sh |
| 地图一直等待 `/map` | 检查发布者；仅看底图使用独立加载，建图/导航检查模块、依赖和生命周期激活 |
| 缺少 map → base_footprint | SLAM 或定位尚未建立全局 TF；导航先等待自动定位完成 |
| 等待扫描时刻 TF | 检查雷达时间戳、真实时钟和 odom/模型/地图 TF 链 |
| 激光或电池样本过期 | 检查对应驱动、USB、供电和 Domain |
| 自动定位失败/相似候选 | 查看定位消息，确认地图对应现场、雷达无遮挡；留出旋转空间后重试，失败时不猜测位置 |
| 低电压停止旋转 | 处理底盘电池/实际供电；静止电压回升不表示带载正常，不要绕过 10.6V 门禁 |
| planner_server 显示 0/未激活 | 等待全局 TF 的激活回调可能使查询超时；先处理定位，不等同于缺包 |
| 模块冲突或外部进程 | 停止对应组合模块或原启动终端的进程，避免重复驱动/地图发布者 |
| 更新后状态仍旧 | 完整更新前后端、重启网关并 Ctrl+F5；单独替换 HTML 不够 |

只读排查（新 Bash 终端先 source 统一环境）：

```bash
ros2 node list
ros2 topic info /cmd_vel -v
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic info /depth_cam/rgb/image_raw -v
```

车端网关日志通常为 `/home/ubuntu/jetauto_cockpit/logs/gateway.log`，静态服务日志为同目录 `frontend.log`；按需启动的 rosbridge 日志为 `/tmp/rosbridge_9090.log`。手动在终端启动时以终端输出为准。

## 测试与实车验证记录

### 本地测试

```bash
python3 -m pip install -r tests/requirements.txt
python3 -m unittest discover -s tests -v
python3 -m playwright install chromium
python3 tests/browser_smoke.py
python3 tests/browser_mapping.py
python3 tests/browser_navigation.py
COCKPIT_BROWSER_AUTO_TEST=1 python3 tests/browser_navigation.py
node tests/test_map_math.js
node --check frontend/cockpit_transport.js
node --check frontend/live_map.js
bash -n backend/start_cockpit_agent.sh
bash -n backend/ros_env.sh
```

最近完成 51 项 Python 测试，以及原控制/视频、地图、备用手动定位/导航、自动定位进度/重试/取消的浏览器测试。浏览器测试拦截实际机器人连接，不驱动机器人。

另在机器人 ROS Domain 42 使用真实 Nav2 与合成传感器、不启动硬件驱动，验证了未知位置自动定位、对称场景拒绝、仅前向扫描配合约 358.7° 旋转融合定位、全部节点 active、目标接收/反馈、暂停/恢复/取消、逐点驻留循环和速度限制。合成测试不证明实车定位或行驶精度。

### 实机检查历史

| 日期 | 有效结论 |
| --- | --- |
| 2026-11-12 | 修复真实 API/遥测连接、视频 topic 斜杠兼容、Bash/Zsh 环境、控制租约、超时零速、按钮更新与停止模块自动加锁问题；另发现 USB/供电导致设备缺失 |
| 2026-11-15 | 补齐 Nav2 依赖，新增 51 包、升级 1 包、无移除；检查时主组件版本为 1.1.20。验证地图、RGB、odom/车体/雷达 TF、关键参数及私有速度路由；保留厂商导航包和地图 |
| 2026-11-21 | 自动定位代码部署。实车完成约 358.6° 旋转，融合匹配 score 约 0.865、候选分差约 0.202；当时 AMCL 沿墙不确定度较大，保持门禁，随后分离自动/人工初始化协方差 |
| 2026-11-21 最终复测 | 修正后完整启动，旋转约 12.5° 时带载电压降到约 10.6V，保护停车；记录平移速度全为零、角速度不超过 0.10rad/s、最后指令零速。最终版本的实车定位成功和导航行驶验收尚未完成 |

检查记录是历史采样，不代表机器人当前实时状态。原网页/RGB及地图 119 × 129 已确认可用；当前有效回波主要在前方约 170°。实测 base_footprint 到 lidar_frame 静态位移约 `(0.09594,-0.00001,0.12840)m`、四元数 `(0,0,0,1)`。地图大小与射线数量可能随地图或配置改变。

安装时还发现无关 Intel RealSense 软件源缺少公钥 `FB0B24895113F120`；ROS/Ubuntu 安装成功，没有修改该无关源。深度点云避障、机械臂外廓与实车最终定位/行驶精度仍待验证。

## 备份与回退

已有车端备份：

- `/home/ubuntu/jetauto_cockpit_backups/20261115-205223`：cockpit、厂商 navigation、地图和原 dpkg 清单。
- `/home/ubuntu/jetauto_cockpit_backups/20261121-auto-localization/cockpit_before.tgz`：自动定位修改前项目。

原始检查证据保留在项目之外，主要目录为：

- `C:/Users/kiefer_k/Desktop/navi/output/cockpit-review-2026-11-12`
- `C:/Users/kiefer_k/Desktop/navi/output/cockpit-deployed-check-20261112-152531`
- `C:/Users/kiefer_k/Desktop/navi/output/nav2-completion-2026-11-15`
- `C:/Users/kiefer_k/Desktop/navi/output/auto-localization-2026-11-21`

