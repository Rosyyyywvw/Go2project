# HandsFree RTK 工作空间

工作空间目录：`/home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws`

本工作空间于 2026-11-02 从 `/home/unitree/catkin_ws` 重命名并移动至此，包含两个原有 ROS Noetic 包：

- `handsfree_rtk`：RTK 驱动，支持普通模式与 NTRIP 差分模式。
- `amap_outdoor_indoor_nav`：地理坐标转换节点与高德定位、路线可视化面板。

## 编译与加载

```bash
source /opt/ros/noetic/setup.bash
cd /home/unitree/Go2W_Project/03_navigation/handsfree_rtk_ws
catkin_make -j2 -l2
source devel/setup.bash
rospack find handsfree_rtk
rospack find amap_outdoor_indoor_nav
```

## RTK 启动入口

加载本工作空间后，按需要选择一种模式：

```bash
roslaunch handsfree_rtk handsfree_rtk.launch
```

```bash
roslaunch handsfree_rtk handsfree_rtk_ntrip.launch
```

运行前检查对应 launch 文件中的串口、波特率及 NTRIP 参数。包内原厂 README 保留原样，其中 `~/catkin_ws` 为安装示例，使用本机时应替换为上方工作空间目录。

## 高德可视化面板

编译并加载环境后，在已有 RTK 驱动旁启动面板。尚未启动地理转换节点时：

```bash
roslaunch amap_outdoor_indoor_nav amap_panel.launch start_geo_layer:=true
```

转换节点已经运行时，省略 `start_geo_layer:=true`，避免重复启动。浏览器打开 <http://127.0.0.1:8088>；局域网访问可追加 `host:=0.0.0.0` 并使用机器人的 IP。

地图需要高德 Web 端（JS API）Key 及对应的安全密钥/白名单配置，可在浏览器会话中设置，或通过面板进程的 `AMAP_JS_KEY`、`AMAP_JS_SECURITY_CODE` 环境变量提供。完整参数和离线界面启动方式见 [面板说明](src/amap_outdoor_indoor_nav/README.md)。

面板在高德地图中显示定位和步行路线预览，并在独立的 FAR `world` 米制视图中显示实际路径、LIO 位置和目标；缺少对应数据时显示等待状态。面板不执行车辆导航、不设置原点；高德路线也不是 FAR 的实际规划路径。GNSS 和 LIO 坐标系完成对齐之前，不将 FAR 路径叠加到高德地图，不可将地图目标直接传给 FAR。定位是否为 RTK Fixed 应检查新鲜 GGA=4，不能仅根据 `NavSatFix` 的 GBAS 状态判断；固定解状态或驱动估算协方差本身都不能证明实测厘米级精度。

迁移后已重新生成 `build` 和 `devel`。迁移前的构建目录、源码校验清单与重新编译日志保存在 [迁移备份](../../backups/handsfree_rtk_relocation_20261102_7gd1gse4/)。
