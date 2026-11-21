# v1.6.1：修复保存地图一直等待 /map

## 实机复查结论（2026-11-15）

在 `192.168.3.73` 检查到：

- 新版地图通道可连接，`mapping_view=true`，雷达仍持续更新；但 `map_revision=0`，没有收到地图。
- ROS 图中 `/map` 没有发布者。`map_501.yaml` 和 `map_501.pgm` 均存在，路径不是问题。
- `nav2_map_server` 及其独立可执行文件存在；缺少 `nav2_amcl`、`nav2_lifecycle_manager`、`nav2_controller`、`nav2_planner`、`nav2_bt_navigator`。
- 旧容器日志显示地图服务等待外部生命周期激活，随后多次报组件资源不存在；当前地图服务的只读状态查询在 2 秒内没有响应。还有两个同名 `nav2_container`，存在组件装载目标冲突的可能。

因此，单纯启动厂家 Nav2 组合 launch 不能保证当前机器发布保存地图。我此前建议通过这个入口加载地图，在本机不完整的导航环境下不够可靠。本版新增独立加载入口。

本次没有修改机器人程序文件、安装包、启停机器人服务、激活地图服务或发送运动指令。

## 更新方法

先在旧页面停车，并停止当前受管的 Nav2/SLAM 组合模块。备份后把部署包中的 `frontend/`、`backend/` 代码覆盖到 `/home/ubuntu/jetauto_cockpit/` 对应目录，保留自己的地图文件、站点数据和配置。

本版新增 `backend/saved_maps.py`、`backend/saved_map_worker.py`，并更新 HTML、地图脚本、网关和地图订阅模块，必须完整更新两端。

在原启动终端停止旧网关后，启动新版：

```bash
cd /home/ubuntu/jetauto_cockpit
ROS_DOMAIN_ID=3 bash backend/start_cockpit_agent.sh
```

打开 `http://192.168.3.73:3000/jetauto_pro_cockpit.html`，按 Ctrl+F5 强制刷新。现有 3000 静态服务可继续使用。

## 加载 map_501

1. 在“已保存地图名称”填写 `map_501`，可以点击“刷新列表”查看可用名称。名称不带 `.yaml` 和目录。
2. 点击 **“加载地图（仅显示）”**，不要通过 Nav2 启动按钮加载。
3. 后端只启动一个独立命名的 `nav2_map_server/map_server`，为这个受管节点依次执行 configure、activate，成功后才返回“已激活”。它不启动底盘、相机、雷达、AMCL 或导航目标。
4. 等待地图出现，点击“适应地图”查看全图。
5. 结束时点击“停止保存地图服务”。重新选择另一个名称并加载，可以切换本网关管理的保存地图服务。

单独查看保存地图不要求底盘、雷达运行。若还在运行 SLAM 或其他实际 `/map` 发布者，加载请求会明确拒绝；不会替换或停止外部地图节点。受管的 SLAM/导航模块与这个独立地图服务互斥。

**加载地图不等于机器人已经定位。** 当前机器缺少 AMCL，平台初始位姿/导航目标也尚未实现。因此可能出现“地图已收到”，同时 TF 提示缺少 `map → base_footprint` 或激光时间变换；此时底图可以正常查看，机器人/激光不会被错误叠加。全套定位导航需要另行补齐。

## 新接口

| 接口 | 作用 |
| --- | --- |
| `GET /api/maps` | 列出 `JETAUTO_MAP_DIR` 中允许加载的 YAML 地图名，附带独立地图服务状态 |
| `POST /api/map/load` | `{"name":"map_501"}`，启动并激活受管的独立地图服务 |
| `POST /api/map/unload` | 只停止本网关创建的保存地图服务 |
| `GET /api/health` | 新增 `saved_map` 状态、错误与最近日志 |

默认地图目录仍为 `/home/ubuntu/ros2_ws/src/slam/maps`。独立服务使用唯一节点名，只切换自身的生命周期，不调用旧 `/map_server` 的状态变更服务，也不接管现存 Nav2 容器。

Nav2 启动入口增加依赖预检查，依赖不完整时在启动任何基础驱动前返回明确错误，避免只拉起半套导航。

## 验证范围

30 项本地 Python 测试、地图几何测试、原有驾驶舱和新版保存地图加载/绘制浏览器测试通过。覆盖服务 configure→activate 顺序、独立节点目标、路径与互斥校验、失败清理、仅加载地图的 UI 请求以及像素显示。

已只读核对实机文件、包、发布者和日志。**本版尚未覆盖到机器人，也未在实机执行加载/激活操作；需部署后验证。** 原始证据在 `C:/Users/kiefer_k/Desktop/navi/output/map-wait-check-2026-11-15/`。
