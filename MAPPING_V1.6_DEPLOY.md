# v1.6 网页实时建图：更新和使用

> 保存地图的加载请使用 v1.6.1 新增独立入口，见 SAVED_MAP_V1.6.1.md；当前机器 Nav2 依赖不完整，不能依靠 Nav2 组合启动来保证地图发布。

本版默认 IP 已改为 `192.168.3.73`。从机器人网页访问时，仍优先使用页面所在主机。

## 需要替换哪些文件

需要同时更新 `frontend/` 和 `backend/` 中的代码，不能仅更新 HTML。部署包顶层为 `jetauto_cockpit_fullstack/`，取其中这两个目录的内容，覆盖到机器人原有 `/home/ubuntu/jetauto_cockpit/` 下对应目录：

```text
/home/ubuntu/jetauto_cockpit/
├── frontend/
│   ├── jetauto_pro_cockpit.html
│   ├── cockpit_transport.js
│   └── live_map.js               新增
└── backend/
    ├── agent_server.py           已更新
    ├── mapping_stream.py         新增
    ├── control_safety.py
    ├── ros_env.sh
    ├── start_cockpit_agent.sh
    └── requirements.txt
```

覆盖前备份原代码，保留已有 `waypoints_store.json` 和自己的配置。无需覆盖 `/home/ubuntu/ros2_ws`，无需重新编译 ROS 包。本版使用机器人已有的 `nav_msgs`、`sensor_msgs`、`tf2_ros`，没有新增 pip 依赖。

## 更新后如何启动

1. 先停车，在原终端停止旧网关（Ctrl+C）。
2. 覆盖上述目录中的代码后，启动新版网关：

   ```bash
   cd /home/ubuntu/jetauto_cockpit
   ROS_DOMAIN_ID=3 bash backend/start_cockpit_agent.sh
   ```

3. 已运行的 3000 静态服务通常无需重启；如果尚未运行，另开终端：

   ```bash
   cd /home/ubuntu/jetauto_cockpit
   python3 -m http.server 3000 --bind 0.0.0.0 --directory frontend
   ```

4. 打开 `http://192.168.3.73:3000/jetauto_pro_cockpit.html`，用 Ctrl+F5 强制刷新。

如果 SLAM 已经运行，可以继续使用，不需要为显示地图重复启动。由旧网关启动的进程在新网关中可能被标记为“外部进程”；这不影响地图观察，新网关不会自动接管或停止它。若尚未启动 SLAM，仍按平台原有组合 launch 互斥要求启动，避免重复开启底盘/相机/雷达。

## 网页上能看到什么

- 占据栅格：白色为空闲、黑色为障碍、灰色为未知，支持地图原点偏移和朝向。
- 红点：激光扫描，已按扫描时间通过完整三维 TF 转换到地图坐标系。
- 蓝色车体和白色箭头：机器人在地图中的位置和朝向；急停时车体变红。
- 状态栏：地图尺寸、分辨率、坐标系及机器人/激光 TF 状态。
- 鼠标拖动平移、滚轮缩放；“适应地图”恢复自动适应地图范围；“居中车体”对准当前有效机器人位置。
- 占据栅格与激光可分别隐藏。默认自动适应范围，手动拖动或缩放后保持用户视角。

TF 缺失或数据过期时，不绘制失效的车体/激光，并显示具体原因。地图连接断开时保留最后底图并明确标为旧数据，重连后恢复。地图内容按更新推送，不会在每个激光帧重复发送整个栅格。

## 常见提示

| 提示 | 检查方法 |
| --- | --- |
| 地图服务未连接 | 确认 `mapping_stream.py` 和 `agent_server.py` 都已更新，并重启了 8000 网关；仅刷新 HTML 不够 |
| 地图订阅尚未就绪/缺少模块 | 查看 `/api/health` 中 `mapping_error`；终端需加载正确 ROS 环境和已有 tf2_ros 工作空间 |
| 等待 `/map` | SLAM 尚未发布地图，查看 SLAM 日志、雷达和里程计状态 |
| 缺少机器人 TF | 缺少地图坐标系到 `base_footprint` 的有效变换，检查 SLAM 和 EKF |
| 等待扫描时刻 TF | 雷达时间戳与 TF 不能对齐；检查发布时钟、TF 链及对应日志 |
| 激光扫描已过期 | `/scan` 没有持续更新，检查雷达节点和 USB 连接 |

`/api/health` 的 `capabilities.mapping_view=true` 代表地图订阅组件初始化成功；实际地图/TF 是否可用以网页状态栏为准。地图通道为只读的 `WS /ws/mapping`，独立于手动控制和遥测连接；地图错误不会主动触发急停或启动运动。

## 已验证范围

2026-11-15 在 `192.168.3.73` 上用临时、只读订阅节点验证了新增地图模块：收到 65×100、0.05m/格地图、机器人位姿和 331 个有效地图坐标激光点，TF 状态正常。验证代码通过标准输入运行，没有上传/覆盖机器人程序文件、启停原有服务或发送运动指令。

本地 22 项 Python 测试、地图几何测试、原有驾驶舱浏览器回归和新增地图像素/交互测试通过；实车采样回放已检查地图渲染。完整新网关和网页尚未部署到机器人，部署后的连续建图显示仍需验收。

Nav2 任务下发、初始位姿、地图点选导航、实车 3D、代价地图和全局规划路径不在本次范围，相关操作继续禁用。
