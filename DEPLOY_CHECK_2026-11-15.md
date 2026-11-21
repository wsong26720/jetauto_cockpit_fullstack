# v1.7.0 车端部署与检查记录

检查日期：2026-11-15，Asia/Shanghai。目标：`ubuntu@192.168.3.73`。用户授权安装依赖、修改并部署代码；使用 `map_501`，实际位置未知。

## 完成内容

- 修改前备份：`/home/ubuntu/jetauto_cockpit_backups/20261115-205223`。cockpit、原 navigation、地图均已归档，保存原 dpkg 清单。
- 安装 navigation2、nav2_bringup、ThetaStar 和相关依赖：新增 51 包，升级 1 包，移除 0 包；Nav2 主组件版本 1.1.20。
- 部署整个 frontend/backend，包括两份导航 YAML 和专用启动文件。没有改动厂商 navigation 包或地图文件。
- 正常停止旧网关后启动新版，OpenAPI 版本为 1.7.0；实际网页已检查，无 JS 异常。
- 运行 Domain 3 网关 8000、网页 3000、rosbridge 9090、导航组合模块及 8080 视频服务。没有配置开机自启；重启机器人后仍需启动网关与网页服务，再从页面启动所需模块。

## 实车只读验证

| 检查 | 结果 |
| --- | --- |
| `/map` | map_server 发布 map_501，网页收到 119 × 129 栅格 |
| RGB | 8080 返回 HTTP 200 MJPEG，检测到 JPEG 帧头 |
| 雷达 | `/scan` 实际收到 382 条射线，frame 为 lidar_frame |
| odom → base_footprint | 实际动态 TF 可用 |
| base_footprint → base_link | 实际静态 TF 可用 |
| base_footprint → lidar_frame | 静态位移约 (0.09594, -0.00001, 0.12840)m，四元数 (0,0,0,1) |
| map → base_footprint | 尚不存在：没有确认实际初始位姿，未伪造或默认设为零 |
| 生命周期 | map_server、amcl、controller_server、smoother_server active；其余等待全局定位完成激活 |
| 控制器 | DWB，X 上限 0.15、Y 0.10m/s、Yaw 0.40rad/s，横向阈值 0.001 |
| 局部代价地图 | odom / base_footprint，2D 障碍层与膨胀层 |
| 全局代价地图 | map / base_footprint，静态层、2D 障碍层与膨胀层 |
| AMCL | OmniMotionModel，set_initial_pose=false，真实时钟 |
| 速度路径 | Nav2 → 私有原始速度 → 平滑器 → 私有导航速度 → 网关 → /cmd_vel |
| /cmd_vel 发布者 | 只有 jetauto_web_cockpit_gateway；订阅者 odom_publisher |
| 电池 | 接入外部供电后仍报告约 9.58～9.76V；新增的 10.6V 驱动保护生效 |
| 页面状态 | 已显示真实地图、未确认定位原因和低电压原因；导航目标与启动任务禁用，初始位姿标注可用 |

planner_server 在等待缺失全局 TF 的激活回调中，生命周期/参数查询可能超时显示 0；没有将该状态描述为实车导航全部就绪。其配置及插件在隔离测试中已成功加载，全部节点 active。

## 软件与隔离 ROS 测试

- 40 项 Python 测试：租约、速度仲裁、超时停车、急停、发布源与驱动检查、地图/TF/路径、Action 任务状态等。
- 三组浏览器测试：原控制和视频回归；地图显示；初始位姿草稿不自动提交、显式确认、目标提交、反馈、暂停确认后恢复、取消。所有机器人请求被模拟，不会驱动物理底盘。
- Python / JS / Shell 语法、DOM 引用与地图几何检查通过。
- 真实页面只读访问验证：地图 119 × 129、RGB MJPEG、未知初始位姿与低电压提示，无页面脚本异常。

在机器人 **Domain 42** 中使用同一版真实 Nav2 与网关，启动参数 `start_drivers:=false`；没有启动硬件节点，不访问串口。合成节点发布 odom、TF、雷达、电池，订阅速度而不驱动物理机器人。结果：

1. 没有初始定位时任务返回 409。
2. 对合成位置提交初始位姿后，AMCL 响应、地图和激光 TF 可用、全部 9 个导航生命周期节点 active。
3. 单点任务被 Nav2 接受并返回距离/耗时反馈，速度通过网关且在限幅内。
4. 暂停等待取消终态进入 PAUSED，恢复重发，取消进入 CANCELED，接收端最终速度为零。
5. 原生 NavigateThroughPoses 任务成功完成；逐点 NavigateToPose 加驻留的两站、两轮任务成功完成。到点完成测试使用合成当前位置附近目标，未模拟实际沿路径行驶。
6. 结束后已清理隔离测试进程，8012 测试网关未继续运行。

早期测试脚本曾因缺失必填字段、未等地图到达、合成 TF 缺少 base_link 而失败，均修正测试夹具后重新验证；最终测试结果 PASS。隔离测试不证明真实 AMCL 匹配正确、实际避障可靠或底盘行驶精度达标。

## 尚需人工确认的条件

实车真实位置与朝向未知，需要操作者在网页地图上标注并明确确认，再核对激光与地图是否重合。底盘电压遥测仍低于阈值，需要检查底盘实际供电/电池，恢复正常遥测后才允许驱动。未向 Domain 3 发布初始位姿或导航目标，未发送非零实车运动命令。

当前 footprint 是待实测参数，机械臂伸出外廓尚未验证；深度点云未接入避障层，3D 和代价地图图层仍禁用。没有完成实际行驶验收。

APT 中另有 Intel RealSense 仓库公钥缺失 `FB0B24895113F120`，未影响本次 ROS 依赖安装，未修改无关仓库。

## 代码、日志与操作说明

本地项目：`C:\Users\kiefer_k\Desktop\jetauto_cockpit_fullstack`。完整操作及 HTTP API 见 `NAVIGATION_V1.7.md`。

本次原始检查证据位于 `C:\Users\kiefer_k\Desktop\navi\output\nav2-completion-2026-11-15`，包含 apt 安装记录、实机参数/TF JSON、隔离测试状态和速度记录、最终部署检查及网页地图截图。车端网关日志：`/home/ubuntu/jetauto_cockpit/logs/gateway.log`。
