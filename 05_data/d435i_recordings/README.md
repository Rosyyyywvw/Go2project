# D435i 真实场景采集

此目录位于 Go2W_Project 项目内，是独立真实目录，不使用 `05_data/rosbags` 的外部软链接。

```text
d435i_recordings/
├── bags/             原始 ROS bag，每段场景一个文件
├── videos/           后续从 bag 转出的 MP4，供 infer_video.py 使用
├── notes/            采集条件、问题时间点及测试集划分记录
├── launch_camera.sh  已验证的 640×480、30 FPS 深度＋彩色配置
└── record.sh         按场景录制彩色图像和 camera_info
```

## 启动相机

若相机已经按上述配置运行，可直接开始录制。需要重启时，先在原相机终端按 Ctrl+C，再在终端 A 执行：

```bash
cd /home/unitree/Go2W_Project/05_data/d435i_recordings
bash launch_camera.sh
```

不要同时运行两个相机驱动。使用已确认的 USB3 连接；`lsusb -t` 应显示相机为 `5000M`。

## 画面方向

当前相机倒装，`orientation.json` 设置 `rotation_degrees: 180`。`launch_camera.sh` 同时启动正向预览节点；在 `rqt_image_view` 的下拉框选择 `/camera/color/image_upright`。相机已运行时，可单独在终端启动预览：

```bash
source /opt/ros/noetic/setup.bash
python3 /home/unitree/Go2W_Project/05_data/d435i_recordings/upright_preview.py
```

原始话题 `/camera/color/image_raw` 仍保持相机原始方向，录制 bag 的命令不变。正向预览仅供画面查看和二维检测，没有匹配的 CameraInfo 或 TF，不用于深度配准、三维投影或标定。相机恢复正装时，将 `orientation.json` 的值改为 `0` 并重启预览。

## 录制试片和正式场景

在终端 B 执行：

```bash
cd /home/unitree/Go2W_Project/05_data/d435i_recordings
bash record.sh 00_test 10
```

先检查保存的试片，再逐段采集正式场景：

| 场景名 | 内容 | 建议时长 |
| --- | --- | --- |
| 01_pedestrian | 单人、多人、横穿、遮挡、背向离开 | 60 秒 |
| 02_bicycle | 停放、推行、安全距离内骑行 | 60 秒 |
| 03_cone | 交通锥近中远距离、画面边缘 | 60 秒 |
| 04_barrier | 水马、隔离栏、施工围挡 | 60 秒 |
| 05_mixed | 人车混合、前进、转弯 | 60 秒 |
| 06_background | 空走廊、墙、树木、玻璃、广告牌 | 60 秒 |
| 07_entrance | 门和入口，预留给 YOLOE | 60 秒 |

例如录制行人场景：

```bash
bash record.sh 01_pedestrian 60
```

如果希望自己决定结束时间，把秒数填为 `0`：

```bash
bash record.sh 01_pedestrian 0
```

该模式不设时长上限。在录制终端按一次 Ctrl+C，等待程序退出并完成 bag 保存，再关闭终端。保存位置仍为 `bags/`。

到达时长上限自动停止；Ctrl+C 可提前结束并保存。ROS master 正常走时、有持续图像时，录制时长约为指定秒数；数据中断或仿真时钟暂停时可按 Ctrl+C 停止。文件名自动包含上海时区时间和纳秒，重复采集会生成新文件。

脚本只录制 `/camera/color/image_raw` 和 `/camera/color/camera_info`。相机驱动仍发布深度，但深度、IMU 不保存在此批 bag 中。

## 检查和后续使用

```bash
source /opt/ros/noetic/setup.bash
rosbag info bags/实际文件名.bag
```

确认两个话题均有消息、时长符合预期。离线回放时，先停止现场相机驱动，再执行：

```bash
rosbag play bags/实际文件名.bag
```

另一终端执行 `rqt_image_view` 并选择 `/camera/color/image_raw` 检查画面。播放 bag 时需有 ROS master，可单独运行 `roscore`。

原始 bag 保留，使用以下命令另行转换为 MP4（无需启动相机或 ROS master）：

```bash
source /opt/ros/noetic/setup.bash
python3 bag_to_video.py bags/实际文件名.bag
```

生成文件保存在 `videos/`，转换核验报告保存在 `notes/`。脚本根据图像时间戳估计帧率，保持原始分辨率、颜色及图像数量，以恒定帧率写入 MP4（MPEG-4 Part 2 编码）。若源图像中断或丢帧，输出不补帧，时长可能与 bag 的实际时间跨度不同。脚本会完整解码生成的视频验证帧数，不覆盖已有输出；中断后残留的 `.partial.mp4` 为未完成文件。

转换默认读取 `orientation.json`，目前自动旋转 180°，输出文件名带 `_upright.mp4`。原始 bag 和已导出的旧视频保留。若某段 bag 是相机正装时拍的，显式指定 `--rotation 0`；导出时应按该段采集时的实际安装方向选择，配置不会自动判断历史 bag 的方向。

MP4 可传至模型所在电脑，供 `infer_video.py` 读取；此工具不执行模型检测。分辨率以 bag 内实际图像为准，录制脚本不会修改相机参数。

640×480、30 FPS、RGB8 未压缩彩色数据每分钟约 1.66 GB，7 段约 11.6 GB，另需留出 MP4 和文件开销空间。录制前检查 `df -h .`。相机固定在实际安装高度和角度，保留近中远距离及不同光照。

在 `notes/` 记录每段拍摄地点、光照、安装高度/角度、目标距离、是否保留为独立测试集。后续标注或微调前，先按整段视频划分训练与测试数据。
