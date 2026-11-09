# 高德定位与路线可视化面板

本包在现有 WGS-84 → GCJ-02 / ENU 转换节点旁增加浏览器面板：高德地图显示机器人定位、定位质量、轨迹及步行路线预览；独立的 FAR 米制视图显示实际规划路径、LIO 位置和目标。Python 3 后台使用标准库 HTTP 服务和 `rospy` 订阅 ROS；无需 rosbridge。

高德路线仅供预览，面板不会执行导航、发布运动指令或设置 ENU 原点。高德道路路线不是 FAR 的实际规划路径。FAR 路径以自己的 `world` 米制坐标独立显示；GNSS 的 `geo_enu` 与 LIO 坐标系尚未完成偏航、平移及外参对齐时，不将 FAR 路径叠加到高德地图，也不能把地图上的路线或目标直接当作 FAR 目标。没有对应导航数据时，FAR 视图显示等待状态。

## 编译和启动

每个新终端先加载 ROS 和本工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd /home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws
catkin_make -j2 -l2
source devel/setup.bash
```

需要实车定位时，按当前设备配置启动已有 RTK 驱动；两个入口选择其一，已经运行时无需重复启动：

```bash
roslaunch handsfree_rtk handsfree_rtk.launch
# 或使用已配置好的 NTRIP 差分模式：
roslaunch handsfree_rtk handsfree_rtk_ntrip.launch
```

在另一个已加载环境的终端启动面板及转换节点：

```bash
source /opt/ros/noetic/setup.bash
source /home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws/devel/setup.bash
roslaunch amap_outdoor_indoor_nav amap_panel.launch start_geo_layer:=true
```

如果转换节点已经运行，使用默认的 `start_geo_layer:=false`：

```bash
roslaunch amap_outdoor_indoor_nav amap_panel.launch
```

打开 <http://127.0.0.1:8088>。需要从同一局域网其他设备访问时，追加 `host:=0.0.0.0`，再访问 `http://机器人IP:8088`；端口可用 `port:=8088` 调整。面板不会自动启动 RTK 驱动或车辆控制节点。

## 高德地图凭据

使用高德开放平台 **Web 端（JS API）** 类型的 Key，并按该 Key 的配置准备安全密钥和访问白名单，参见[高德 JS API 准备说明](https://developer.amap.com/api/javascript-api-v2/getting-started)。浏览器访问所用的主机名或 IP 应与平台允许的访问配置一致。地图加载和路线查询需要浏览器能够访问高德服务。

可以直接在面板设置中填写 Key 和安全密钥，凭据仅保存在当前浏览器会话；也可以在启动面板的终端预先设置 `AMAP_JS_KEY`、`AMAP_JS_SECURITY_CODE` 两个环境变量。不要把真实凭据写入源码、YAML 或 launch 参数。后台不使用 RTK 驱动中的 NTRIP 账户。

面板还会自动读取本机私有配置 `~/.config/go2w/amap_panel.json`（设置了 `XDG_CONFIG_HOME` 时使用对应配置目录），字段为 `amap_key`、`amap_security_js_code`；该文件应设为仅本人可读写，修改后重启面板。环境变量优先于本机配置；已由后台提供的字段在浏览器设置中显示为只读。可用 `--credentials-file` 或 ROS 私有参数 `credentials_file` 指定其他配置路径。

如果路线查询提示 `USERKEY_PLAT_NOMATCH`，应在高德控制台添加 **Web 端（JS API）** 类型的 Key；其他平台类型的 Key 不能直接用于此网页。`INVALID_USER_SCODE` 表示安全密钥配置需要检查。新申请的 JS API Key 需配套 `securityJsCode`，参见[准备说明](https://developer.amap.com/api/javascript-api-v2/getting-started)及[错误码说明](https://lbs.amap.com/api/javascript-api-v2/guide/abc/errorcode)。仅成功初始化 JS 地图对象不代表路线服务已经通过鉴权。

无 Key 或无 ROS 数据时可先查看面板和状态；不会生成模拟位置或虚构轨迹。仅预览界面可运行：

```bash
python3 /home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws/src/amap_outdoor_indoor_nav/scripts/amap_panel.py --standalone --host 127.0.0.1 --port 8088
```

## 数据和坐标

| launch 参数 | 默认话题 | 用途 |
| --- | --- | --- |
| `gnss_topic` | `/handsfree/rtk/gnss` | WGS-84 `sensor_msgs/NavSatFix` |
| `gcj02_topic` | `/amap_nav/gcj02_fix` | 转换后的高德坐标 |
| `raw_topic` | `/handsfree/rtk/raw` | 原始 NMEA，读取 GGA 定位质量 |
| `enu_topic` | `/amap_nav/enu_pose` | 本地 ENU 位置 |
| `origin_topic` | `/amap_nav/enu_origin` | 已锁定的 ENU 原点 |
| `geo_status_topic` | `/amap_nav/geo_status` | 转换节点状态文字 |
| `path_topic` | `/viz_path_topic` | FAR `visualization_msgs/Marker`，`ns=global_path` 的 `LINE_STRIP` |
| `odom_topic` | `/lio/odom` | LIO `nav_msgs/Odometry` |
| `goal_topic` | `/goal_point` | FAR 目标 `geometry_msgs/PointStamped` |
| `reached_topic` | `/far_reach_goal_status` | FAR 到达状态 `std_msgs/Bool` |

`stale_after_sec` 默认为 3 秒，`max_track_points` 默认为 3000。话题参数可按已有节点配置覆盖。`start_geo_layer:=true` 时，面板的 GNSS、原始 NMEA 及地理输出话题参数会同步传给转换节点；使用已经运行的转换节点时，需将面板订阅参数与其输出配置保持一致。

`NavSatFix.status=STATUS_GBAS_FIX` 本身不能证明 RTK Fixed：当前驱动可能将 Fixed 和 Float 都映射到 GBAS。应结合原始 GGA 的质量字段判断，`4` 为 RTK Fixed、`5` 为 RTK Float。

当前驱动中的位置协方差按 GGA 类别查表估算，不能当作实测定位精度。GGA=4 表示固定解状态，也不单独证明已经达到厘米级精度。

`config/geo_coordinate_layer_real.yaml` 采用源码支持的 `quality_mode: rtk_fixed_only`，要求 1 秒内收到 GGA=4，再接受 GNSS 数据；同时采用 `origin_mode: service` 和 `publish_gcj02: true`。因此，未锁定原点时仍能显示通过质量检查的 GCJ-02 定位，但 ENU 与原点暂缺是正常状态。该面板不会调用设置原点的服务。若已有转换节点使用其他质量策略，以该节点配置为准。

## 后端回归验证

在工作空间根目录运行以下命令，无需启动 ROS 或实车。后端的 12 个回归用例覆盖数据状态、HTTP 接口与本机凭据配置：

```bash
cd /home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws
python3 -m unittest discover -s src/amap_outdoor_indoor_nav/tests -v
```
