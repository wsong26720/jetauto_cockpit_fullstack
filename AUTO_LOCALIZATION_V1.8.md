# v1.8 自动启动定位

日期：2026-11-21。默认地图 `map_501`。本版在启动导航后自动搜索机器人在地图中的位置和朝向，不再要求人工标注初始位姿。

## 操作

1. 打开 `http://192.168.3.73:3000/jetauto_pro_cockpit.html`，Ctrl+F5 刷新。
2. 确认地图名称和周围环境一致，整车及机械臂周围已留出旋转空间，再启动“Nav2 自主定位与导航”。不要同时启动独立底盘、雷达、相机或保存地图显示服务。
3. 观察“自动定位”状态：等待数据 → 整图搜索 → 必要时低速原地旋转 → 融合搜索 → AMCL 验证 → 自动定位成功。
4. 等待定位、TF 和导航节点就绪后再点击目标导航。成功前导航目标禁用。无需操作地图上的手动标注工具。
5. 可以点击“取消自动定位”停止定位，或点击“重新自动定位”重试。急停仍可用。手动标注保留为可选备用入口，自动定位时禁用。

## 实现与边界

先用 `/scan` 及 `base_footprint ← lidar_frame` 的完整 TF 将扫描转换到车体坐标，在整张已知地图上搜索 X/Y 和 360° 朝向。粗搜索约 0.20m/10°，候选精搜索约 0.05m/2°，检查有效回波、端点匹配、射线自由空间和相似候选分差。

当静止扫描存在相似候选且已允许旋转时，以不超过 **0.10rad/s** 原地转约一圈，不发送平移指令；按 odom 将不同朝向的扫描融合到同一参考位姿，再进行整图匹配。不是把任意位置或地图原点写成初始位姿。

只有候选质量和唯一性检查通过后，才自动发布 `/initialpose`。自动候选的初始协方差根据局部匹配峰形设置，位置方差下限 0.0125m²、偏航方差下限为 5° 的方差；人工粗标注仍使用原来的 0.25m²/15° 设置。随后调用 AMCL `/request_nomotion_update`，验证新 AMCL 响应、协方差、扫描与地图的一致性、TF 和位置稳定性。至少连续 8 个有效新样本、稳定 3 秒才确认。定位成功不代表底盘/电压或导航生命周期一定已就绪，仍保留原有就绪检查。

旋转速度通过网关单独授权通道输出，导航速度输入不能覆盖定位旋转指令。缓存条件超过 0.3 秒未更新或定位指令超过 0.3 秒未续约即输出零速。电压、驱动、激光和 TF 不满足条件，检测到其他 `/cmd_vel` 发布者、可见旋转包络 0.43m 内有障碍、异常平移超过 0.10m 或朝向跳变时停止。旋转最多 85 秒；允许旋转时总定位上限 150 秒，仅静止模式上限 60 秒。失败或取消后不开放导航，不会自动反复旋转。

0.43m 包络检查基于当前车体尺寸估计和可见激光回波，不能覆盖雷达盲区、机械臂伸出、桌面等高于激光面的障碍。启动前仍需留出整车与机械臂的旋转空间。用户已在本次会话中授权低速原地旋转并确认周围已清空。

完全对称的房间、重复走廊、地图与现场变化大、激光遮挡等情况，可能无法唯一确定位置。系统会保持停车并说明原因。匹配检查是保守启发式，不是对定位正确性的数学保证，不能承诺任意环境下一定定位成功。

## API 与状态

| 接口 | 用途 |
| --- | --- |
| `POST /api/localization/auto`，请求体 `{}` | 启动/重试自动定位；活动导航任务或手动控制存在时拒绝 |
| `POST /api/localization/cancel`，请求体 `{}` | 取消自动定位并撤销定位旋转速度授权 |
| `GET /api/navigation/status` | `localization` 字段给出阶段、消息、错误、候选质量与旋转角度 |
| `WS /ws/telemetry` / `GET /api/health` | `nav_state.localization` 同上；`control.source=localization` 表示定位旋转控制 |

阶段：IDLE、WAITING、MATCHING、ROTATING、VERIFYING、SUCCEEDED、FAILED、CANCELED。`readiness.initial_pose_source=automatic` 表示自动验证通过，`auto_pending` 表示候选已提交但尚未通过验证。`initial_pose_confirmed` 同时用于自动验证和可选的人工确认，不再表示必须由人点击。

`match` 中的 score、runner_up_score、margin、hit_ratio、free_ratio 用于解释匹配质量；这些数值不是定位正确率或概率。margin 小于 0.06 的相似候选不会接受。

## 配置和依赖

`JETAUTO_AUTO_LOCALIZATION=true` 默认自动启动定位；设为 false 关闭自动触发，但仍可通过 API 主动请求定位。`JETAUTO_AUTO_ROTATE=true` 允许必要时原地旋转；标准 `start_cockpit_agent.sh` 默认启用，设为 false 可强制仅静止定位。直接运行 `agent_server.py` 时，未设置 AUTO_ROTATE 默认不旋转。

新增计算依赖 numpy、scipy，本次机器人均已安装。需要时使用 Ubuntu 的 `python3-numpy`、`python3-scipy`，无需修改底层电机固件或厂商控制包。

新增 `backend/global_scan_matcher.py` 与 `backend/automatic_localization.py`。已更新导航桥、只读传感器取样、网关仲裁和网页状态。应同时部署完整 frontend/backend 并重启网关；不能只替换 HTML。

## 验证和回退

本次新增检查覆盖非对称地图的全局定位、旋转地图原点、对称地图拒绝、扫描不足、取消、协方差以外的匹配检查、运动授权隔离和失效停车。另使用隔离 ROS Domain 42、真实 Nav2 与合成传感器验证自动定位成功与拒绝，以及前向扫描配合原地旋转融合定位。没有在隔离域启动物理驱动。

修改前机器人备份：`/home/ubuntu/jetauto_cockpit_backups/20261121-auto-localization/cockpit_before.tgz`。本地修改前副本及验证记录：`C:\Users\kiefer_k\Desktop\navi\output\auto-localization-2026-11-21`。本次没有修改地图或厂商控制包。

已有任务接口、速度限制、地图与导航参数说明见 `NAVIGATION_V1.7.md`；本文件覆盖其中“启动必须人工标注初始位姿”的旧流程。

本次实际部署与验收边界见 DEPLOY_CHECK_2026-11-21.md：最终实车复测因带载电压下降而自动停车，尚未完成最终版本的实车定位成功验收。
