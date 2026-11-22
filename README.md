# JetAuto Pro Web 驾驶舱 2.0

通过浏览器驾驶 JetAuto Pro、查看 RGB 和激光、建立二维地图，并在已保存地图上定位和导航。本项目包含网页与车端网关。底盘运动学、电机控制和里程计由厂商驱动负责，全局路径由 Nav2 Theta* 规划，局部轨迹由 DWB 控制器选择。

**适配环境**为 Jetson Orin NX Super、Ubuntu 22.04、ROS 2 Humble、G4 二维激光雷达及 Orbbec Astra Pro Plus。机器人当前配置地址为 `192.168.3.73`，ROS Domain 默认 `3`。更换地址后通过新 IP 访问网页，或在页面连接设置中修改。网关提供驾驶和启动服务的权限，部署在可信局域网内使用。

## 安装、升级与启动

车端部署目录为 `/home/ubuntu/jetauto_cockpit`。从旧版升级时，先在网页结束任务、停止受管模块，退出网关，再备份当前部署目录。把发布包里的 `backend/`、`frontend/` 和本 README 放入部署目录。保留自己的 `backend/waypoints_store.json` 和地图。已有代价地图配置的用户应合并 `backend/config/nav2_params.yaml` 中自己的参数。2.0 的到点参数在另一个文件 `backend/config/nav2_controller.yaml`，该文件需要一起更新。

车端需要厂商工作空间中的 `controller`、`ros_robot_controller`、`peripherals`、`slam`、机器人模型、G4 和 Orbbec 驱动，以及对应的设备规则。RGB 视频还需能从该 ROS 环境执行 `ros2 run web_video_server web_video_server`。配置好 Humble 软件源后，用以下命令安装通用依赖并启动后端。

```bash
sudo apt-get install -y --no-remove ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-theta-star-planner ros-humble-rosbridge-server python3-numpy python3-scipy
cd /home/ubuntu/jetauto_cockpit
python3 -m pip install -r backend/requirements.txt
ROS_DOMAIN_ID=3 bash backend/start_cockpit_agent.sh
```

第二个终端启动网页：

```bash
cd /home/ubuntu/jetauto_cockpit
python3 -m http.server 3000 --bind 0.0.0.0 --directory frontend
```

打开[驾驶舱](http://192.168.3.73:3000/jetauto_pro_cockpit.html)，升级后按 **Ctrl+F5**。页面标题及[网关 OpenAPI](http://192.168.3.73:8000/openapi.json) 的 `info.version` 均应为 `2.0`。

| 端口 | 服务 | 如何确认 |
| --- | --- | --- |
| 3000 | 网页静态文件 | 页面显示实车模式 |
| 8000 | HTTP 网关、控制和遥测 WebSocket | [健康状态](http://192.168.3.73:8000/api/health) 中检查 `ros_initialized`、`control_ready`、`control_issues` |
| 9090 | rosbridge | 启动网关时自动启动或复用。已有监听进程须使用相同 ROS Domain |
| 8080 | `web_video_server` | 在模块管理启动视频服务，再打开 [RGB 视频](http://192.168.3.73:8080/stream?topic=/depth_cam/rgb/image_raw&type=mjpeg&quality=80) |

网关和受管子进程通过 `backend/ros_env.sh` 加载 Humble、厂商硬件配置和工作空间。网页启动模块时无需手工逐个 source。另开 Bash 终端查询 ROS 时执行 `source /home/ubuntu/jetauto_cockpit/backend/ros_env.sh`。厂商的 Zsh 配置不适合直接在 Bash 中加载。

## 日常操作

| 任务 | 网页操作 |
| --- | --- |
| 手动驾驶与看视频 | 在实车模式启动“底盘运动与 EKF 控制栈”。需要视频时再启动深度相机及 Web 视频服务。底盘、电池遥测和控制通道就绪后使用摇杆，松开或页面失焦即停止续发 |
| 手动驾驶建图 | 停止导航、独立地图服务及分开运行的基础驱动，启动“2D SLAM 在线建图”。等待地图和激光出现，驾车探索，用地图保存按钮输出到允许目录，如 `/home/ubuntu/ros2_ws/src/slam/maps/map_502`，不带扩展名 |
| 查看已有地图 | 停止其他地图发布源。在地图名称中输入 `map_501`，点击“加载地图（仅显示）”。独立 map_server 激活后即可看底图 |
| 自动定位与导航 | 卸载独立地图并停止 SLAM 及分开启动的基础驱动。选择地图，启动“Nav2 自主定位与导航”。该组合包含底盘、相机、雷达及导航节点。等待定位和导航就绪，再用“指哪走哪”按下选位置、拖动选朝向、松开提交 |
| 多站、驻留与恢复 | 在地图拾取站点或输入坐标。连续经过模式使用 `NavigateThroughPoses`。逐站巡航使用连续的 `NavigateToPose`，支持驻留及循环。任务暂停后，等待状态变为 `PAUSED` 才能恢复。开始另一任务前先取消暂停的任务 |

独立底盘启动器是 `ros2 launch controller controller.launch.py`，它提供速度转换、里程计和 EKF。单独启动 `ros_robot_controller` 只启动底层板驱动。若模块显示为“外部进程”，请在其原启动终端或所属组合模块中停止。模拟模式的三维场景及模拟传感器仅供界面演示。实车 RGB 使用 `/depth_cam/rgb/image_raw`，本版的导航避障使用二维 `/scan`，原始 `16UC1` 深度图需要额外的显示转换。

## 定位、精度与控制

**启动定位。** 网关按激光采集时刻查询雷达到底盘的完整 TF。全局搜索使用地图距离场、最多 120 个采样点，依次搜索位置和朝向，再对有明确候选分差的结果做栅格内精修。候选需通过障碍命中、射线经过自由区域、相似候选分差和局部峰形检查。AMCL 接收初始位姿后，网关继续验证至少 8 次独立观测、至少 3 秒的稳定性。地图重复、遮挡或回波稀少时，页面会给出定位失败原因。

**补充扫描。** 标准启动脚本允许低速原地旋转补充扫描，最大角速度 0.10 rad/s。旋转前、旋转中都按运行中的车体外廓、余量及额外 7 cm 间距检查雷达回波。机械臂应处在已测量的行驶姿态。若只允许静止匹配，可在启动网关前设置 `JETAUTO_AUTO_ROTATE=false`。若要人工标注，先取消自动定位，在地图拖出实际车头朝向并确认。2.0 会请求静止 AMCL 更新，等待连续三次稳定更新，并检查与人工标注的偏差是否在 0.20 m、12° 以内再开放任务。

**跟踪与到点。** 网页保留地图拾取和车体采样的原始坐标、朝向，列表显示时才取整。DWB 进入末端转向的位置窗口与 Nav2 目标检查器统一为 **0.10 m**，目标偏航容差为 **0.08 rad（约 4.6°）**。这是对地图估计位姿的判定。实车地面误差还取决于地图、传感器外参、AMCL 和麦轮打滑。[Humble RotateToGoal 实现](https://api.nav2.org/nav2-humble/html/rotate__to__goal_8cpp_source.html)

**控制链。** 手动或导航速度都经过网关仲裁。导航链路为 Nav2 → `/cockpit/nav_raw` → 速度平滑器 → `/cockpit/nav_cmd_vel` → 网关 → `/cmd_vel` → 厂商底盘。位置使用 `map` 坐标；TF 主链为 `map → odom → base_footprint`。网页/API 目标朝向用逆时针度，ROS 角速度用 rad/s。

| 控制条件 | 默认值 |
| --- | --- |
| 手动平移 / 旋转上限 | X、Y 各 ±0.20 m/s，旋转 ±0.50 rad/s |
| 导航平移 / 旋转上限 | X −0.10～+0.15 m/s，Y ±0.10 m/s，旋转 ±0.40 rad/s |
| 电池与速度有效期 | 电池至少 10.6 V，样本小于 5 s。速度超时 0.5 s |
| 导航门槛 | 唯一有效底图、采集时间有效的激光、车体 TF、AMCL 观测、就绪的 Nav2 生命周期和 Action、有效代价地图及外廓 |
| 接管 | 网关软急停撤销运动授权。解除急停后，需要新的控制输入。外部 ROS 发布者由其自身管理 |

## 代价地图和目标预检查

地图下面的“代价地图参数配置”同时显示保存值和从 ROS 读回的运行值。全局图使用 `map`，局部滚动图使用 `odom`，由 TF 对齐显示。红色 254 表示实际障碍，橙色 253 表示车体中心禁入，紫色 255 表示未知，蓝色 1–252 表示不同程度的通行代价。

| 参数 | 随包默认值 | 作用 |
| --- | --- | --- |
| 车体轮廓、余量 | 0.50 × 0.44 m，单侧 0.02 m | 两层共用。按车轮、车身及机械臂收拢后实测外廓填写有序凸多边形 |
| 膨胀半径、衰减系数 | 0.45 m、3.0 | 全局和局部可分别配置。系数增加会让障碍外围代价更快下降 |
| 栅格分辨率 | 0.05 m | 全局静态层可采用原始地图分辨率，面板另列实际栅格值 |
| 雷达标记 / 清障距离 | 6 m / 8 m | 清障距离须大于等于标记距离 |

修改后点击“保存，下次启动生效”。网关校验后原子替换项目的导航 YAML，前一份保存在同目录 `nav2_params.yaml.previous`。在机器人停稳、任务已取消后，停止并重新启动导航模块，等面板读回运行值。保存时出现 409 表示其他页面或程序已经修改配置，需重新读取。

导航预检查同时检查起点、每个目标中心以及目标朝向下带余量的外廓。Nav2 在执行阶段搜索整条路径。即使预检查通过，路径阻塞、定位变化或局部控制失败仍可能导致任务中止。

## HTTP 与 WebSocket

基地址是 `http://<机器人IP>:8000`。完整模型和 HTTP 调试入口见[车端接口文档](http://192.168.3.73:8000/docs)。

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
| `POST /api/navigation/precheck` | `{mode,waypoints,loop_count}`，只读检查起点和各目标，返回 valid、issues、start、targets，不下发 Action |
| `GET /api/navigation/costmap-settings` | 返回 settings、revision、runtime、pending_changes、restart_required 和 warnings；运行值缺失时 restart_required 为 null |
| `POST /api/navigation/costmap-settings` | `{revision,settings}`；只保存本项目导航配置，不改运行节点、不自动重启、不发送运动指令；冲突返回 409，参数无效返回 422 |
| `POST /api/navigation/task` | `{mode,waypoints,loop_count}`，提交前后端及每次底层 Action 前重复检查 |
| `POST /api/navigation/pause` | `{}`，关闭速度授权、取消 Action，等终态进入 PAUSED |
| `POST /api/navigation/resume` | `{}`，PAUSED 且就绪时重发未完成目标 |
| `POST /api/navigation/cancel` | `{}`，关闭授权并请求取消任务 |
| `GET /api/waypoints` | 读取后端站点存储，返回 `{waypoints:[...]}` |
| `POST /api/waypoints` | 请求体为 WaypointItem 数组，保存到 `backend/waypoints_store.json` |

导航模式为 `single`、`through_poses`、`waypoints`。每次 1–100 个点，循环 1–10 次，逐站驻留 0–300 秒。以下请求体可先提交给只读的 `/api/navigation/precheck`。在任务接口使用时应填写现场核对过的坐标。

```json
{"mode":"single","loop_count":1,"waypoints":[{"id":"station-a","name":"A","x":1.2,"y":0.4,"yaw_deg":90,"dwell_sec":0}]}
```

代价地图配置 POST 的请求体为 `{"revision":"<读取所得哈希>","settings":{...}}`。先 GET 取得完整 `settings`，只变更需要的字段后原样提交。加载地图使用地图名，保存地图使用允许目录内不带扩展名的输出路径。

| WebSocket | 数据 |
| --- | --- |
| `/ws/telemetry` | 约 1 Hz 推送健康状态、硬件、电池、任务反馈和模块列表 |
| `/ws/mapping` | 约 10 Hz 推送地图状态、扫描、位姿和路径。底图、全局和局部代价栅格按各自修订号传输 |
| `/ws/control` | 发送 `{linear_x,linear_y,angular_z}`，读取就绪或拒绝原因。断开撤销该客户端的手动租约 |

## 配置、源码与排查

| 位置 | 负责内容 |
| --- | --- |
| `frontend/jetauto_pro_cockpit.html` | 页面、地图选点、任务与模块交互 |
| `frontend/cockpit_transport.js`、`live_map.js`、`costmap_settings.js` | 连接与心跳、地图栅格渲染、参数表单 |
| `backend/agent_server.py`、`control_safety.py` | HTTP/WebSocket、进程管理、遥测及速度仲裁 |
| `backend/navigation_bridge.py`、`automatic_localization.py`、`global_scan_matcher.py` | Action 生命周期、初始位姿验证、激光匹配 |
| `backend/mapping_stream.py`、`costmap_stream.py`、`costmap_settings.py`、`navigation_precheck.py` | 地图与 TF 采样、参数持久化、外廓检查 |
| `backend/saved_maps.py`、`saved_map_worker.py` | 已保存地图目录和独立 map_server 生命周期 |
| `backend/cockpit_navigation.launch.py`、`backend/config/` | Nav2 节点组合，AMCL、Theta*、DWB、代价地图与平滑器参数 |
| `tests/` | Python 单元/API 测试、地图数学和浏览器操作回归 |

`JETAUTO_WS`、`THIRD_PARTY_WS` 和 `ORBBEC_WS` 指定厂商及外设工作空间。`JETAUTO_MAP_DIR` 默认取 `$JETAUTO_WS/src/slam/maps`。`JETAUTO_AGENT_PORT` 可覆盖 8000。`JETAUTO_AUTO_LOCALIZATION=false` 关闭启动时自动定位，仍可从网页主动请求。网页样式和演示 3D 使用外部 CDN。

在车端打开单独的终端并加载 `backend/ros_env.sh` 后，可用下面的只读命令区分地图、定位、雷达与目标执行的问题。

```bash
ros2 topic info /map -v
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /amcl
ros2 action info /navigate_to_pose
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
```

| 现象 | 检查位置 |
| --- | --- |
| 网页能开，模块或驾驶按钮不可用 | 确认“实车直连”、IP、8000 和 9090。看 `/api/health` 的具体原因，再检查 `/dev/rrc`、`/cmd_vel` 订阅者、电池与急停 |
| 一直等待真实地图或激光 TF | 检查 `ros2 topic info /map -v`、`ros2 topic info /scan -v`、ROS Domain 和消息时钟。保留一个地图发布源，确认雷达设备 `/dev/lidar` |
| 全局定位相似候选过多 | 核对地图和雷达视野，在特征明显的位置重试，或确认一次人工位姿。定位期间查看页面给出的 AMCL 与扫描匹配状态 |
| ROS 状态 6（ABORTED） | 先读任务错误和预检查原因。253 表示离障碍太近。若起终点通过，检查 `planner_server`、`controller_server` 日志、路径及局部代价图 |
| 保存配置后运行值仍未知 | 检查受管导航是否启动和参数服务是否可读；对比 `global_costmap`、`local_costmap` 的运行值 |
| 地图在前端变了，版本或功能仍旧 | 检查 `/openapi.json`，重启更新后的网关并强制刷新网页。回退时停受管模块、停网关，恢复升级前项目备份后重启 |

## 验证与已知边界

2.0 已通过 87 项 Python 测试、地图几何测试，以及手动控制、视频、地图、导航、代价预检查和参数配置的浏览器操作测试。回归覆盖迟到的代价图响应、重复底图、过期激光、移动后的 AMCL 观测和人工标注一致性。

在开发机上，用同一张 5 cm 非对称合成地图生成 12 个已知位姿的扫描，对 1.0.2 与 2.0 分别执行全局定位。两版都接受了 12 个位姿。以下为这一组样本的中位数：

| 指标 | 1.0.2 | 2.0 |
| --- | ---: | ---: |
| 初始位置误差 | 1.85 cm | 1.18 cm |
| 初始朝向误差 | 0.48° | 0.12° |
| 建立匹配器并完成一次搜索 | 196 ms | 146 ms |

在 Jetson 上以独立 ROS Domain（87 或 88）、回环网络、真实 Humble Nav2 和模拟底盘运行闭环测试。雷达观测由地图光线投射生成，模拟底盘按网关输出速度积分。人工给定起点后，在每张图上向相距 0.8 m、朝向 24° 的目标导航。下表记录各条件下一次成功任务结束时，目标与模拟底盘真值的偏差。

| 地图 | 调整前，朝向容差 0.15 rad | 发布配置，朝向容差 0.08 rad |
| --- | --- | --- |
| 非对称合成房间 | 8.3 cm，8.9° | 4.7 cm，4.7° |
| 已保存地图 `map_501` | 24.7 cm，8.8° | 27.3 cm，4.5° |

这两次对照都观察到了终点朝向改善。`map_501` 的位置误差仍较大。开发过程中的一次早期模拟使用了不同的地图边界射线处理，位置误差达到 37 cm。两组导航试验每个条件只有一个样本，无法据此判定真实车辆的位置精度有改善。

在 `map_501` 使用静止自动定位时，前两名候选分差仅为 0.009，低于接受阈值 0.06。网关提示环境有相似位置，保持未定位状态。该试验关闭了补充旋转。隔离测试期间也观察到连续重启后地图服务配置偶发超时，更换隔离 Domain 重试后通过。若车端启动后一直未就绪，可先检查地图发布者和两个生命周期管理器的日志。

渲染复用代价图画布、像素缓冲及调色表。导航就绪检查只查询扫描时刻的 TF，绘图时才投影激光点。搜索分批处理候选位姿以限制临时数组大小。上述计时是开发机匹配耗时，网页帧率、Jetson 算力余量及真实地面行驶误差尚无对比测量。

在开发机安装测试依赖后执行：

```bash
python3 -m pip install -r tests/requirements.txt
python3 -m unittest discover -s tests
node tests/test_map_math.js
python3 -m playwright install chromium
python3 tests/browser_smoke.py
python3 tests/browser_mapping.py
python3 tests/browser_navigation.py
COCKPIT_BROWSER_AUTO_TEST=1 python3 tests/browser_navigation.py
COCKPIT_BROWSER_COSTMAP_TEST=1 python3 tests/browser_navigation.py
python3 tests/browser_costmap_settings.py
```

旧版已在此车上验证 RGB、地图、代价图和导航目标拒绝原因。2.0 的实车地面验收应以独立测量为准：在标记点记录目标位置与车头角度，多次从不同方向抵达，用实测终点计算位置和朝向误差。机械臂伸出后的轮廓、麦轮地面滑移和雷达平面之外的障碍仍需现场核对。
