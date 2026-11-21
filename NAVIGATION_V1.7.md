# JetAuto 网页导航 v1.7.0

> v1.8 已增加启动自动定位，当前操作请以 AUTO_LOCALIZATION_V1.8.md 为准。本文保留 v1.7 接口及历史检查记录。

更新与实机检查：2026-11-15。机器人 `192.168.3.73`，Ubuntu 22.04、ROS 2 Humble，使用 `map_501`。

## 当前状态与使用步骤

已安装缺失的 Nav2 依赖，并将前后端部署到 `/home/ubuntu/jetauto_cockpit`，重启了网关。实车没有确认初始位姿；没有向实车发送移动目标。

1. 打开 `http://192.168.3.73:3000/jetauto_pro_cockpit.html`，按 Ctrl+F5 强制刷新。
2. 确认电池遥测有效且至少 10.6V。接入外部供电后，本次底盘遥测仍低于阈值，因此驱动保护不会自动绕过。
3. 使用实车模式，地图名填 `map_501`。直接启动“Nav2 自主定位与导航”组合模块。它包含底盘、EKF、机器人模型 TF、雷达、相机、地图服务、AMCL 与 Nav2；不要再单独启动相同基础模块。
4. 如果“保存地图服务（仅显示）”、SLAM 或独立底盘/雷达/相机在运行，先停止它们，再启动导航。8080 视频服务可以独立运行。
5. 在地图上找到机器人实际位置，选择“标定初始位姿”，按住鼠标从实际位置向车头方向拖动。橙色标记只是草稿；点击导航面板的“确认标注为机器人实际位置与朝向”才提交。
6. 等待提示“定位、TF 与导航节点已就绪”。核对机器人标记和激光轮廓与真实环境重合。AMCL 收到消息或协方差较小，都不能代替操作者核对位置。
7. 选择“指哪走哪”并在地图可通行区域拖出目标朝向，或拾取站点后点击“启动任务”。指哪走哪在松开鼠标时即提交单点导航任务。
8. 暂停会取消当前底层 Action，并等待其终态；显示 PAUSED 后才可恢复。恢复重新下发尚未完成的目标。取消任务不会自动恢复。

机器人在地图中的位置未知时，请先结合房间墙角、门口等环境特征确认位置；不要随意点击原点或把地图中心当作当前位置。

## 已安装依赖与配置

本次执行：

```bash
sudo apt-get install -y --no-remove \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-theta-star-planner
```

安装新增 51 个包，更新 1 个包，无移除。实机 Nav2 主组件版本为 1.1.20。APT 更新时另外一个 Intel RealSense 源存在签名公钥缺失；ROS/Ubuntu 源与本次安装成功，未改动该无关源。

配置归属本项目：

| 文件 | 用途 |
| --- | --- |
| `backend/cockpit_navigation.launch.py` | 独立节点启动、生命周期管理与速度话题重映射 |
| `backend/config/nav2_params.yaml` | 地图、AMCL、规划、代价地图、行为与速度平滑 |
| `backend/config/nav2_controller.yaml` | DWB 控制器、目标容差及速度限制 |

保留厂商 navigation 包。修正了厂商配置中的仿真时钟值，并让控制器同时加载基础参数与控制器参数，避免局部代价地图回退到错误的 frame。

| 参数 | 当前值 |
| --- | --- |
| 时钟 | 全部 `use_sim_time=false` |
| AMCL 模型 | `nav2_amcl::OmniMotionModel` |
| AMCL 初始位姿 | `set_initial_pose=false`，等待人工明确确认；不自动写入零位姿 |
| TF 主链 | `map → odom → base_footprint → base_link / lidar_frame` |
| 局部 / 全局代价地图 frame | `odom` / `map`；车体 `base_footprint` |
| 2D 障碍信息 | `/scan`，障碍层与膨胀层；全局额外包含静态地图层 |
| 全局规划 / 局部控制 | ThetaStar / DWB 全向控制 |
| 自动导航速度 | X -0.10～+0.15m/s，Y ±0.10m/s，Yaw ±0.40rad/s |
| 到点容差 | 位置 0.15m，偏航 0.20rad |
| 配置 footprint | 长 0.50m、宽 0.44m 的矩形，额外 padding 0.02m |

footprint 为暂用参数，尚未测量实际整车外廓，也未验证机械臂伸出时的碰撞包络；速度与容差是初始调试值，尚未进行实车行驶调参。深度点云尚未接入本版避障层，不能据此判断桌面等高于雷达扫描平面的障碍可避让。

导航启动后，地图服务和 AMCL 先激活；全局代价地图等待初始定位产生 `map → odom`，后续导航节点才能全部激活。未知初始位姿时 planner_server 的生命周期查询可能超时显示 0，页面会明确保持未就绪。

## 速度路径和驱动条件

```text
Nav2 controller / behavior
  → /cockpit/nav_raw
  → velocity_smoother
  → /cockpit/nav_cmd_vel
  → 网关任务授权、限速、手动/导航仲裁、软急停
  → /cmd_vel
  → odom_publisher → 底层板驱动
```

Nav2 接受网关任务后才开启导航速度授权。没有本网关任务时，私有导航速度被忽略。手动控制有独立租约；导航执行期间非零手动请求返回 409，需先暂停/取消后接管。零速请求和软急停会停止本网关导航授权并请求取消任务。

导航驱动要求地图、已确认并响应的 AMCL 位姿、可用 TF、全部导航节点 active、Action 服务和私有速度通道就绪。检测到绕过网关的 `/cmd_vel` 发布者时禁止本网关导航。底盘额外要求设备、实际订阅者、电池发布者和 5 秒内且不低于 10.6V 的电池样本。

导航速度超过 0.5 秒未更新时输出零速；定位或就绪条件持续丢失会取消当前任务。手动租约同样在 0.5 秒超时后停车。接收目标或取消确认超时会保留未确认状态，需成功停止/重启导航后才能继续。

这些保护只覆盖本网关控制路径；软急停不能替代硬件急停，也不能阻止外部程序绕过网关直接驱动底盘。

## HTTP API

基础地址：`http://192.168.3.73:8000`，OpenAPI：`/docs`。接口供可信局域网使用，当前没有认证与 TLS。

### 启动导航

`POST /api/launches`

```json
{"module_id":"navigation","action":"start","map_path":"map_501"}
```

`map_path` 是允许目录中的地图名称，不是任意文件路径。网关转换为 `JETAUTO_MAP_DIR/map_501.yaml`。支持 start/stop/restart。只管理自己启动的进程；已有外部驱动或地图源会明确报冲突。

### 初始定位

`POST /api/localization/initialpose`

```json
{"x":1.2,"y":-0.4,"yaw_deg":90,"confirmed":true}
```

坐标单位米，参考系 `map`，角度为逆时针度数。以上只是字段示例，不是该机器人的实际位置。点必须位于地图已知可通行栅格，AMCL 需 active；活动任务或手动租约存在时返回 409。接口发布 `/initialpose`，位置协方差为 0.25m²，偏航为 15° 的方差；返回 waiting_for_amcl，不能立即视为定位成功。

### 下发任务

`POST /api/navigation/task`

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
| `single` | 恰好一个点，调用 `/navigate_to_pose`，不执行逐点驻留 |
| `through_poses` | 原生 `/navigate_through_poses` 连续经过点列，不执行每点 dwell |
| `waypoints` | 网关逐点调用 `/navigate_to_pose`，每点成功后执行 dwell，支持循环 |

逐点模式没有调用原生 `/follow_waypoints`，以便支持每点不同的驻留时间及可恢复进度。站点数 1～100，loop_count 1～10，dwell_sec 0～300 秒。坐标和角度必须有限；目标点需在地图内且为已知、占用值小于 65 的格。该检查不代替 Nav2 的足迹、膨胀层或路径可达性判断。

提交成功返回 `{ok:true,task_id,state:"pending"}`。HTTP 成功只表示提交，随后可能被 Nav2 拒绝或执行失败；以前端真实反馈/终态为准。

### 状态、暂停、恢复与取消

| 接口 | 行为 |
| --- | --- |
| `GET /api/navigation/status` | 任务状态、readiness、生命周期、AMCL 确认和协方差、反馈、错误与 ROS 结果 |
| `POST /api/navigation/pause` | 立即关闭速度授权，取消 Action，等待进入 PAUSED |
| `POST /api/navigation/resume` | 只允许 PAUSED 且就绪，重新提交未完成目标，保留尚未完成的驻留时间 |
| `POST /api/navigation/cancel` | 关闭速度授权、请求取消任务；不自动恢复 |
| `POST /api/estop?engage=true` | 网关软急停，取消本网关任务并请求零速 |

暂停、恢复、取消请求体均为 `{}`。任务上下文保存在内存中，网关重启后不恢复任务，必须重新确认初始位姿。

`status_text`：IDLE、PENDING、EXECUTING、DWELLING、CANCELING、PAUSED、CANCELED、SUCCEEDED、ABORTED。status_code 用于 UI，其中 PAUSED 与 CANCELED 都是 5；请同时读取 paused/status_text。ros_result_status 是底层 Action 结果，4 成功、5 取消、6 中止。

反馈包含 `distance_remaining`（米）、`estimated_time_remaining` / `navigation_time`（秒）、`number_of_recoveries`、从 0 开始的 `current_waypoint_index`、从 1 开始的 `current_loop`、`dwell_remaining` 和 waypoints。暂停是取消并重新下发，并非 Nav2 内置“冻结/继续”动作。恢复次数等反馈以当前底层 Action 为单位；ETA 为 0 也可能表示尚无可用估计，不表示必然到达。

`GET /api/health` 和 `WS /ws/telemetry` 的 nav_state 包含同一状态。`WS /ws/mapping` 增加最近 3 秒内、地图 frame 一致的 `/plan` 折线。代价地图和 3D 显示仍未接入。

常见错误：409 未定位/未就绪/控制冲突，423 急停锁定，422 参数或地图点无效，503 组件/服务不可用。

接口字段核对依据为实机 `ros2 interface show` 与 [Nav2 Humble Action 文档](https://api.nav2.org/humble/actions/navigation/)，不使用其他 ROS 版本新增的消息字段。

## 验证与回退

40 项 Python 测试及三组拦截所有实车连接的浏览器测试通过。另在机器人 **ROS_DOMAIN_ID=42** 启动不含硬件驱动的真实 Nav2，配合合成激光/里程计/电池与速度接收端验证了：未定位拒绝目标，确认定位后全部节点 active，单点接收与反馈、暂停/恢复/取消、限速和取消零速，多点完成，以及逐点驻留两轮完成。合成测试不代表实车定位或行驶精度验收。

实车 Domain 3 已验证 odom 到车体、车体到激光 TF、实际 `/scan`、地图服务、AMCL 和关键参数；map 到 odom 仍等待真实初始定位。完整部署检查记录见 `DEPLOY_CHECK_2026-11-15.md`。

修改前备份：`/home/ubuntu/jetauto_cockpit_backups/20261115-205223`，包含 cockpit、厂商 navigation、地图压缩包及原包清单。若需回退，先在网页停止受管模块、在网关启动终端退出网关，再将备份解压到单独目录核对，替换 cockpit 后启动旧网关。地图和厂商 navigation 本次未改动，无需覆盖它们；APT 包无需为代码回退而卸载。
