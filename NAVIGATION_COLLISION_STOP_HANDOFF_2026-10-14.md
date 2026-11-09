# Go2W 导航反复停车问题交接（2026-10-14）

**2026-10-21 路径更新：** 本文保留历史排查记录。当前导航统一使用 `03_navigation/local/autonomous_exploration_development_environment`；旧工作区已退出启动流程，文内旧路径仅供历史对照。当前启动命令见 [go2](go2)，工作区说明见 [WORKSPACE_ISOLATION.md](WORKSPACE_ISOLATION.md)。

**2026-10-15 22:08 最新运行排查：** 用户已于 22:02 启动回退配置。FAR 目标链路正常，主要卡在 localPlanner：90 帧路径中 88 帧只有原点，前进速度始终为零。冻结重放显示旧高度窗接纳了新外参下的地面候选与其他低位点，原近簇也影响剩余路径。此次不是新增保护分支；正式参数保持不变。证据与修复边界见 [Goalpoint 新一轮排查](diagnostics/goal_no_response_20261015-220730/REPORT.md)。

**2026-10-15 当前配置回退：** 按用户要求恢复 10/11 最后保留的导航参数与保护前实现，保留本次新标定并同步雷达 TF。194 项启动参数中 192 项与基准一致，另两项保留新外参与项目路径；本周新增的 20 项参数已移除。局部规划编译、独立 ROS 验证通过；生产节点未重启，下次启动加载。详见 [回退记录与备份](diagnostics/restore_lastweek_20261015-215219/REPORT.md)。下文的保护行为和点统计为此前配置的历史结果。

**2026-10-15 21:45 外参更新：** 已把本次 LI-Init 的最终 Refinement R、t 写入 Super-LIO，平移为 `(0.015117,-0.024535,0.200686) m`；下次启动加载，尚未导航复测。时间偏差 `-0.041418 s` 未应用。旧值备份及校验见 [更新记录](diagnostics/extrinsic_update_20261015-214528/UPDATE.md)。历史候选点统计仍对应各自录制时的外参。

**2026-10-15 历史对照补充：** 已提取上周 26 组、本周 10 组启动参数。本周外参更换，且 pathFollower 从上周不直接订阅点云变为直接碰撞保护；10/12 重标定前的原始包已含同位置附近 ring 4 近点。完整取值、变化时间与判断边界见 [历史参数核查](diagnostics/parameter_history_20261015/REPORT.md)，正式参数未改。

## 结论先行

在 FAR RViz 发布 GoalPoint 后，目标**已经进入 FAR**，FAR 持续发布 `/way_point`，`localPlanner` 持续发布非空 `/path`。底盘程序也在订阅 `/cmd_vel`。狗没有移动，是 `pathFollower` 的碰撞保护反复把 `/cmd_vel` 置零：日志常见 `Collision stop: front 0.08–0.14 m (threshold 0.50 m), swept=1`。不能将此问题简单归因于 GoalPoint 未送达、路径失连或底盘未连接。

**尚未查明触发碰撞保护的近点究竟来自地面、车体、真实近障，还是其中几类的组合。** 不要在未确认点的物理来源之前扩大自体过滤框、调低 `0.5 m` 安全距离，或绕过碰撞保护。

## 环境与启动方式

- ROS1 Noetic；项目目录 `/home/unitree/Go2W_Project`。**2026-10-15 已将自主导航工作区从 hsh 软链接替换为项目内独立副本**：`03_navigation/autonomous_exploration_development_environment`，后续修改该目录不会同步修改 hsh 原目录。部分其他子目录仍为符号链接，例如 FAR 工作区指向 `/home/unitree/farplanner_ws`。复制校验与编译环境迁移见 [工作区隔离说明](WORKSPACE_ISOLATION.md)。
- 总启动脚本：[go2w_start.py](go2w_start.py)。当前运行实例由该脚本在 2026-10-14 启动；运行状态可用 `python3 go2w_start.py status` 查看。
- 局部规划启动文件：[local_planner copy.launch](03_navigation/autonomous_exploration_development_environment/src/local_planner/launch/local_planner%20copy.launch)。FAR 启动文件：[far_planner_go2w.launch](03_navigation/farplanner_ws/src/far_planner-melodic-noetic/src/far_planner/launch/far_planner_go2w.launch)。
- RS32 `/rslidar_points` → `rs_converter` `/velodyne_points` → Super-LIO `/lio/cloud_world`（`world` 坐标系）和 `/lio/odom` → FAR `/way_point` → `localPlanner` `/path` → `pathFollower` `/cmd_vel` → `/unitree_ros_control`。
- `pathFollower` 和 `localPlanner` 都直接订阅 `/lio/cloud_world`；当前 `useTerrainAnalysis=false`，安全检查没有使用地形分类结果。
- `go2w_start.py` 已向 ROS master 加载 Go2W URDF 到 `/robot_description`；默认 FAR RViz **没有** RobotModel 显示，也没有真实关节状态。不要把默认姿态的轮子位置当作实物位置。

## 最近一次复现的现场证据

以下是 2026-10-14 当前运行实例的只读检查；没有为诊断发布 GoalPoint 或运动指令。

| 环节 | 观察结果 |
| --- | --- |
| GoalPoint | `/goal_point` 的发布者是 `/far_rviz`，订阅者包括 `/far_planner` 和 `/pathFollower`。FAR 日志出现 `GP: *********** new goal updated ***********`。`/goalpoint`（无下划线）不是本实例的话题。 |
| FAR | `/way_point` 约 2.4 Hz，消息为 `world` 坐标系；`/far_reach_goal_status=false`。FAR 日志也反复出现 `dynamic obstacle detected`，但这条日志本身不能证明现场真有移动障碍物。 |
| 局部规划 | `/path` 约 10 Hz，`vehicle` 坐标系，最近采样为 101 个姿态。 |
| 定位与点云 | `/lio/odom` 与 `/lio/cloud_world` 均约 10 Hz，时间戳新鲜；IMU 桥日志约 500 Hz。 |
| 命令与底盘接口 | `/pathFollower` 约 89 Hz 发布 `/cmd_vel`，采样到的线速度与角速度都是 `0`；`/unitree_ros_control` 正在订阅。不能据此判断 F1/底盘执行模式，因为目前没有非零命令到达它。 |
| 停车日志 | `Collision stop: front 0.05–0.30 m (threshold 0.50 m), swept=1, turn clearance 0.36–0.55 m, turn allowed=0` 反复出现。偶尔 `front≈1.9 m`，仍可能因 `swept=1` 或转向范围被占而停车。 |
| 后退保护 | 曾记录 `Front and turn blocked; starting guarded reverse up to 0.80 m`，紧接着 `Retreat stopped: progress 0.00/0.80 m, rear clear=0`。这次没有实际后退 0.8 m。 |

当前运行日志目录（下次启动会变化）：

- `/home/unitree/.local/state/go2w_start/logs/20261014-183357/local_planner.log`
- `/home/unitree/.local/state/go2w_start/logs/20261014-183357/far_planner.log`
- `/home/unitree/.local/state/go2w_start/logs/20261014-183357/imu.log`

`local_planner.log` 在目标发布前有 `FAR waypoint missing or stale`，**目标进入 FAR 并开始持续发布路点后**转为反复 `Collision stop`。不要把先前的路点缺失日志当作当前主要故障。

## 点云、坐标与地面的已知事实

1. `/rslidar_points` 与 `/velodyne_points` 的 XYZ 数据在匹配扫描中一致；转换器没有凭空生成近点。把一个触发点按当前 LI-Init 外参和里程计逆变换回 RS32 坐标后，能在原始点云中找到约 `1 mm` 误差的对应点。外参见 [M2DGR.yaml](02_slam/Super-LIO/src/super_lio/config/M2DGR.yaml)。这只能说明坐标变换链自洽，**不能证明外参的实物标定正确**。
2. `pathFollower` 使用的虚拟车体中心由 `/lio/odom` 和 `sensorOffsetX=-0.02557 m` 算出；车体坐标约定为 `x` 向前、`y` 向左、`z` 向上。下面所有坐标都是**相对该虚拟车体/里程计高度**，不是相对地面的高度。
3. 车体前边界为 `vehicleLength/2=0.395 m`；横向检测半宽为 `vehicleWidth/2+safetySideMargin=0.36 m`。`FrontDistance` 对 `x≥0.395 m、|y|≤0.36 m` 的点取最近 `x−0.395 m`。在当前几乎静止时停车阈值为 `safetyClearance=0.5 m`。
4. 当前固定高度窗口是 `minRelZ=-0.45 m`、`maxRelZ=+0.35 m`，严格排除 `z≤-0.45 m` 的点。[pathFollower.cpp](03_navigation/autonomous_exploration_development_environment/src/local_planner/src/pathFollower.cpp) 的点云处理在约第 230–285 行，碰撞决策在约第 497–765 行；几何函数在 [collision_guard.h](03_navigation/autonomous_exploration_development_environment/src/local_planner/include/local_planner/collision_guard.h)。
5. 用户移动狗几米后，曾见车体右前低位点约 `(x=0.49–0.51, y=-0.31～-0.34, z=-0.48～-0.51) m`。用户认为这可能是地面。**这组点低于 `-0.45 m`，不会进入当前停车碰撞检查。** 此前把它称为“车轮/自体回波”的判断已撤回。
6. 最近一次目标无反应期间，进入前方碰撞区的最近点在多帧中约为 `(x=0.47–0.54, y=0.00–0.13, z=-0.45～-0.22) m`。例如 `(0.53, 0.13, -0.438) m` 给出约 `0.135 m` 的前边界间隙；`(0.479, 0.007, -0.227) m` 给出约 `0.084 m`。40 帧中 24 帧有距离前边界不足 `0.5 m` 的点，和停车日志相符。其他帧也可能因扫掠轨迹或原地转向检查停车。
7. 在一次 30 帧采样中，附近较低回波约为 `z=-0.58 m`，进入检查的近点通常高出约 `0.14–0.30 m`。这**不足以断言**它们是真障碍或自体回波：局部地面起伏、地面反射异常、遮挡、车体部件和标定误差仍需区分。用户现场观察前方 1 m 内没有明显实物障碍。

## 当前配置与已撤回的尝试

- `vehicleLength=0.79 m`，`vehicleWidth=0.52 m`，`safetyClearance=0.5 m`，`safetyReactionTime=0.3 s`，`safetyDecel=0.25 m/s²`，`safetySideMargin=0.1 m`。运动时停车阈值还包含反应距离和制动距离。
- `retreatDistance=0.8 m`、`retreatSpeed=0.15 m/s`。只有后方整条通道被观测且足够安全时才允许后退；本次后方检查失败并在 `0 m` 停止。
- 保留的旧自体过滤框很窄：`x=[0.38,0.44]`、`y=[-0.24,-0.17]`、`z=[-0.42,-0.28]`，`selfFilterEnabled=true`。它不覆盖最近的 `(0.53,0.13,-0.438)` 等点。
- 曾根据未经充分证实的“自体回波”假设增加四个过滤框；在用户指出右前低点可能是地面后，**四个新框及配套代码已经撤回并重新编译**。不要依据旧对话把它们当作当前生效配置。当前运行节点没有为此重启或放宽安全检查。
- 目前用固定 `minRelZ` 代替地面分类：高度跨过 `-0.45 m` 的地面点有可能被当障碍；反过来，直接下调该阈值也可能漏掉低矮真实障碍。

## 给接手 agent 的建议工作顺序

1. **只读复现和留证。** 先确认 `go2w_start.py status`、`rostopic info /goal_point /way_point /path /cmd_vel` 和最新日志目录。录制短 rosbag：`/rslidar_points /velodyne_points /lio/cloud_world /lio/odom /way_point /path /cmd_vel`，必要时加 `/joy`、`/far_reach_goal_status`。不要为录制而发布新的目标或速度。
2. **定位每一帧实际触发的点。** 严格复用 `pathFollower` 的车体原点、Yaw 变换、高度窗口、旧自体框和 `collision_guard` 判定，输出触发点的世界坐标、车体坐标、原始 RS32 对应点、ring/intensity、邻域密度与持续时间。分别统计 `FrontDistance`、`SweptPathBlocked`、`TurnDistance`，避免只看 `front` 一项。
3. **确认物理来源。** 将原始点投到实物测量的车体/轮子范围，检查当前高架 RS32 的真实安装外参与标定；比较机器人换位置、转向或改变姿态前后同一簇点的车体坐标。若要用 URDF 辅助，注意默认 RViz 没有真实关节状态，URDF 的雷达安装位置也未确认与当前高架安装一致。对地面应按局部地面高度和连续性判断，不能把“相对里程计高度低”直接等同于地面。
4. **再设计修复。** 只有确认自体回波后才限定其几何过滤范围；只有确认地面穿过固定高度门限后才考虑局部地面估计。对点数、时间连续性等噪声门限要验证不会漏检细小真实障碍。保留 `0.5 m` 前边界安全距离和输入失连停车逻辑。
5. **先离线回放，再受控上车。** 用录到的点云验证修复能消除误停，同时以真实近障样本验证仍会停车。当前 GoalPoint 可能仍然有效；修改过滤或重启 pathFollower 后一旦放行，可能立即输出非零速度。因此在现场确认底盘安全状态前不要重启或热更新避障参数。

本交接记录的是观察与可复现的代码行为，不预设最终物理原因。

## 18:48 追加实测排查

详细证据和离线复现工具见 [实测排查报告](diagnostics/collision_stop_20261014/REPORT.md)。本轮没有修改运行参数、导航源码或重启节点。

- 用户确认狗站立，雷达中心离地约 70 cm，雷达架在车体上方，沿 x 方向未伸出车头。
- 16.5 秒 bag 中有 165 帧世界点云：104 帧前方间隙不足，132 帧仅扫掠初始矩形就有占用，139 帧转向圆内有点。侧面/后侧点可以解释 `front≈1.9 m` 仍停车。
- 候选触发点与原始 ring 0、1、2、4 回波对应，误差全部小于 1 mm；本轮匹配扫描的转换 XYZ 未增加误差。
- 周围主要低平面约为相对 z `-0.564 m`，前方候选点高出它约 `0.113–0.344 m`。单纯的平地跨越 `-0.45 m` 高度门限不足以解释本段数据；自体、近距异常回波、局部高起实物和实物外参仍需核对。
- 本轮全部线速度仍为 0，但出现了短时非零角速度，峰值 `0.228 rad/s`；里程计未显示明显转向。这更新了前文“采样都是零”的观察，底盘模式与短脉冲响应可另行只读核查，不能由此直接判断 F1 或连接故障。
- 驱动实际配置和日志为 `RSHELIOS`；“RS32”是项目称呼，不能据此修改驱动型号配置。

## 21:40 第三轮采集与当前运行状态

用户将位置、朝向改为与前两次不同后，agent 执行 `python3 go2w_start.py start --no-drive`，采集约 22.7 秒、226 帧世界点云及原始雷达、IMU、里程计等。完整数据与对照见 [第三轮报告](diagnostics/collision_stop_round3_20261014-213908/REPORT.md)。

- 采集前旧实例已全部停止，ROS master 不可用；新启动日志实例为 `20261014-213910`，路径为 `/home/unitree/.local/state/go2w_start/logs/20261014-213910/`。**当前仅感知与规划运行，底盘订阅没有启动。**
- 本轮没有发布目标或运动指令。路径是单姿态停止路径，控制日志报 FAR 路点缺失；这不能用来判断近障已消失。相同几何离线复算中，154/226 帧前方间隙不足，203/226 帧扫掠初始矩形被占，211/226 帧转向区域被占。
- 安全参数与 18:48 基线一致。前左、前右侧、正前中间簇的车体坐标中位数跨轮变化约 9–10 mm，原始雷达坐标中位数变化约 2–8 mm；环境回波分布已改变。更支持车体或雷达相关，仍不区分真实部件回波与异常回波。
- 正前中间高位簇再次出现，相对本轮低平面约高 33 cm，不能直接归为轮顶。
- LIO 已重新初始化；两轮 `world` 坐标原点不同，不能直接比较。现有可复算的基线只有 18:48 bag，更早一轮没有保存在诊断目录的 bag。

## 22:13 雷达轴向已知实物验证

用户将雪糕筒放在车头前约 1 m 后，agent 只读采集 12.2 秒。122/122 帧出现稳定的、上窄下宽的原始回波 `(1.355,0.043,-0.379) m`，确认当前雷达 **+X 朝车头、+Y 朝左侧、+Z 基本向上**；不再仅凭 frame 名或外参假设前向。详情见 [轴向报告](diagnostics/lidar_axes_cone_20261014-221248/REPORT.md)。全部记录速度为零，底盘订阅仍未启动，没有发布目标或运动指令。

21:40 与本次里程计显示约 35 cm/3.64° 变化，比较使用未中断的 LIO world 对齐后数据；这两轮可使用同一 world，18:48 与 21:40 之间重启的 world 仍不能直接相比。本次轴向验证不等同于精确外参标定：厂家 Helios 原点定义在中心轴、底座上方 63.5 mm，平移需按该原点实测。

## 2026-10-15 原始 UDP 已证实短距离记录

用户录制了 20:50:26–20:50:38 的 pcap，含 17755 个 MSOP 和 11 个 DIFOP。原始距离字节 `00 c8` 按当前 RSHELIOS 协议对应 0.500 m，设备通道索引 27（ring 4）的角度表为竖直 -43.71°、水平修正 +4.44°，重算得到 `(0.396129,0.011906,-0.345504) m`。独立 Python 计算与实际 SDK C++ 离线解码在参考点 6 cm 邻域得到相同 298 点，XYZ 最大差 4.59e-8 m，SDK 未报解码错误。详见 [UDP 报告](diagnostics/near_raw_check_20261015-204209/UDP_REPORT.md)。

这排除了 TF、IMU 外参和后级 ROS 点云转换作为短距离数值的生成来源；没有证明真实反射物或硬件故障原因。下一项应核对设备测量/光路、厂家 RSView 同包解码和受控实物对照，不能直接扩大过滤框。用户确认此时仍站立、安装未改；20:46 原始点云主要低平面在雷达下方约 65.6 cm，参考近点约高出该平面 33.4 cm，但此拟合不是与 pcap 同步的地面真值。
