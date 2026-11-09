# 当前导航工作区（2026-10-21）

导航统一使用项目内的真实目录：

`/home/unitree/Go2W_Project/03_navigation/local/autonomous_exploration_development_environment`

`go2w_start.py` 和手动启动命令 [go2](go2) 均使用这份工作区。`system_real_robot.launch` 启动局部规划和本工作区的地形分析；FAR 使用 `start_terrain:=false`，消费现成的地形图。LIO、车体 TF 和两个 RViz 窗口分别启动。

local 的规划和地形分析参数保留原值。为接入 Super-LIO，两个地形图的坐标系名称改为 `world`；本工作区原有的 `vehicle_simulator.rviz` 改用 `world`、`/lio/cloud_world` 和 FAR 目标点工具。导航不再依赖旧工作区的配置或程序。

编译和加载环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/unitree/Go2W_Project/03_navigation/local/autonomous_exploration_development_environment
catkin_make -j2 -l2 -DCATKIN_ENABLE_TESTING=OFF
source devel/setup.bash
rospack find local_planner
rospack find terrain_analysis
rospack find terrain_analysis_ext
```

旧工作区、修改前的启动文件，以及 local 原有的构建目录保存在 [本次备份](backups/local_only_20261021_rl5ierv9/)。FAR 仍使用现有独立工作区；其 catkin 环境改为继承 local 工作区。

以下保留 2026-10-15 的历史记录，其中的旧导航路径不再作为启动或编译入口。

## 历史记录：hsh 导航工作区独立副本（2026-10-15）

`/home/unitree/Go2W_Project/03_navigation/autonomous_exploration_development_environment` 已从目录软链接替换为**项目内真实目录**。它不再指向 `/home/unitree/hsh/autonomous_exploration_development_environment`；后续编辑项目内源码、launch、配置和 Git 状态，只影响此副本。

完整复制保留原有源码、未提交修改、未跟踪文件及 `.git` 历史。初始复制共 1467 个常规文件，约 900 MB（十进制），逐文件 SHA-256 与原目录一致；已检查副本与源文件没有共享 inode。hsh 原目录的文件内容和 Git 状态保持不变。

## catkin 编译路径

原工作区的 `build`/`devel` 含有 hsh 绝对路径。复制后将这两个目录的旧副本保存到 [generated_before_relocation](diagnostics/detach_hsh_20261015-201757/generated_before_relocation/)，在项目目录重新生成编译环境。**`catkin_make -j2 -l2` 全部目标编译通过，退出码 0。** 不要使用该备份目录中的构建命令或 setup 文件。

后续从项目副本编译和加载环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/unitree/Go2W_Project/03_navigation/autonomous_exploration_development_environment
catkin_make -j2 -l2
source devel/setup.bash
rospack find local_planner
```

已验证 `local_planner`、`joy`、`terrain_analysis`、`terrain_analysis_ext` 和 `loam_interface` 均解析到项目副本，环境变量不含 hsh 路径。`go2w_start.py` 也已确认选择项目内的 `devel/setup.bash`，无需改变选择逻辑。本次不启动或重启机器人节点。

## 范围与留证

本次只解除解析到 `/home/unitree/hsh` 的软链接；FAR、传感器、SLAM、Unitree 和 rosbag 等原有非 hsh 链接保留。导航副本内的 `src/CMakeLists.txt` 仍使用 catkin 创建的标准 `/opt/ros/noetic/.../toplevel.cmake` 链接，与 hsh 无关。

- [复制结果](diagnostics/detach_hsh_20261015-201757/COPY_RESULT.json)
- [完成结果](diagnostics/detach_hsh_20261015-201757/RESULT.json)
- [原目录文件校验清单](diagnostics/detach_hsh_20261015-201757/source_manifest_before.json)
- [隔离检查](diagnostics/detach_hsh_20261015-201757/ISOLATION_CHECK.json)
- [项目内环境和启动路径检查](diagnostics/detach_hsh_20261015-201757/ENVIRONMENT_CHECK.json)
- [剩余 hsh 链接扫描（0 处）](diagnostics/detach_hsh_20261015-201757/FINAL_LINK_SCAN.json)
- [原目录已有 Git 修改](diagnostics/detach_hsh_20261015-201757/source_git_status_before.txt)
- [重新编译日志](diagnostics/detach_hsh_20261015-201757/build.log)
