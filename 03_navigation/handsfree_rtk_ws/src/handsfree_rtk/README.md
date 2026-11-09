[简体中文]|[English](README_EN.md)  

**[点击查看在线教程](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvEwyKv4gQ1mDA)**  

# T-RTK UM982 使用说明（ROS 环境）  

## 1. 安装  
### 1.1 安装依赖  
使用如下指令完成串口驱动的依赖安装  
```sh
pip install pyserial  # python2 安装
pip3 install pyserial  # python3 安装
```
### 1.2 编译
handsfree_rtk 包主要由python驱动脚本文件、usb挂载点udev规则和launch文件组成  

首先创建一个工作空间（这里工作空间名称可以自定义，这里默认为catkin_ws）  

```bash
cd & mkdir -p catkin_ws/src
```

在我们的在线文档中下载 handsfree_rtk.zip ,解压到 ~/catkin_ws/src 中  

在解压后的 handsfree_rtk 文件夹中打开终端，运行以下指令进行安装。  

```bash
cd ~/catkin_ws/src/handsfree_rtk/
bash auto_install.sh
```

## 2. 使用  
可使用如下指令将 handsfree_rtk 的 ros 设置到.bashrc文件中，便于后期使用，“~/catkin_ws/”是工作空间，用户请根据实际情况修改。  

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 2.1 handsfree_rtk.launch  

- **功能**：  
   RTK模块驱动程序，解析GNGGA、GNRMC数据。  
- **串口名称**：  
   `/dev/HFRobotRTK`（可通过`~port`参数修改）  
- **发布话题**:
  - `handsfree/rtk/raw` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html))，原始NMEA协议数据
  - `handsfree/rtk/gnss` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))，解析后的GNSS定位数据（包含定位状态、经纬度、海拔、协方差矩阵）
  - `handsfree/rtk/speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的地速数据，单位 m/s
  - `handsfree/rtk/cog` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的地面航向数据（COG，表示设备运动方向，相对于正北顺时针旋转的角度，0°=正北，90°=正东，180°=正南，270°=正西），单位 °
  - `handsfree/rtk/heading` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的真航向数据（Heading，表示设备姿态方向，通常为双天线基线方向，独立于运动方向），单位 °
- **启动命令**：  

   ```bash
   roslaunch handsfree_rtk handsfree_rtk.launch
   ```

### 2.2 handsfree_rtk_ntrip.launch  

- **功能**：  
   NTRIP客户端驱动，实现基站-移动站差分数据传输。自动将移动站的GGA数据上传至NTRIP服务器，并将接收的RTCM差分数据发送给移动站。
- **串口名称**：  
   `/dev/HFRobotRTK`（可通过`~port`参数修改）
- **接收/发送**：  
  - 向串口发送：`GPGGA 1`指令（触发移动站输出GGA数据）  
  - 向NTRIP服务器发送：移动站的GGA观测数据  
  - 从NTRIP服务器接收：RTCM3.x差分数据流  
- **发布话题**:
  - `handsfree/rtk/raw` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html))，原始NMEA协议数据
  - `handsfree/rtk/gnss` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))，解析后的GNSS定位数据（包含定位状态、经纬度、海拔、协方差矩阵）
  - `handsfree/rtk/speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的地速数据，单位 m/s
  - `handsfree/rtk/cog` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的地面航向数据（COG，表示设备运动方向，相对于正北顺时针旋转的角度，0°=正北，90°=正东，180°=正南，270°=正西），单位 °
  - `handsfree/rtk/heading` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))，解析后的真航向数据（Heading，表示设备姿态方向，通常为双天线基线方向，独立于运动方向），单位 ° 
- **启动命令**：  
   ```bash
   roslaunch handsfree_rtk handsfree_rtk_ntrip.launch
   ```
- **关键参数**：  
  ```xml
      <!-- NTRIP服务器配置 -->
      <arg name="ntrip_server" default="120.253.239.161"/>
      <arg name="ntrip_port" default="8002"/>
      <arg name="ntrip_username" default="$(optenv NTRIP_USERNAME)"/>
      <arg name="ntrip_password" default="$(optenv NTRIP_PASSWORD)"/>
      <arg name="ntrip_mountpoint" default="$(optenv NTRIP_MOUNTPOINT)"/>
  ```

### 2.3 NTRIP CORS账号配置  

这些参数用于配置RTK差分定位服务中的NTRIP客户端，连接国家或商业机构提供的CORS（连续运行参考站系统）服务。以下是各参数的详细解释：  

| 参数名称 | 示例值 | 说明 |
| --- | --- | --- |
| **ntrip_server** | `120.253.239.161` | NTRIP服务器的IP地址或域名，指向提供差分改正数据的CORS中心服务器 |
| **ntrip_port** | `8002` | NTRIP服务使用的TCP端口号（常用端口：2101/8001/8002） |
| **ntrip_username**   | `$NTRIP_USERNAME` | 认证用户名（由CORS服务商提供，用于识别用户身份和计费） |
| **ntrip_password**   | `$NTRIP_PASSWORD` | 认证密码（与用户名配对，用于服务访问鉴权） |
| **ntrip_mountpoint** | `RTCM33_GRCE` | 挂载点名称（指定获取哪种类型/区域的差分数据流） |

---

## 3.通用特性  

1. **GNGGA 语句状态映射**：  

   ```python
    0: ("Invalid Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    1: ("GPS Fix (SPS)", NavSatStatus.STATUS_FIX, 1.0),
    2: ("DGPS Fix", NavSatStatus.STATUS_SBAS_FIX, 0.5),
    3: ("PPS Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    4: ("RTK Fixed", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    5: ("RTK Float", NavSatStatus.STATUS_GBAS_FIX, 0.1),
    6: ("Estimated", NavSatStatus.STATUS_FIX, 5.0),
    7: ("Manual", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    8: ("Simulation", NavSatStatus.STATUS_FIX, 10000.0)
   ```

2. **GNRMC 语句状态映射**  

   ```python
      'A': 'Autonomous Mode',
      'D': 'Differential Mode', 
      'E': 'INS Mode',
      'F': 'RTK Float',
      'M': 'Manual Input Mode',
      'N': 'No Fix',
      'P': 'Precision Mode',
      'R': 'RTK Fixed',
      'S': 'Simulator Mode',
      'V': 'Invalid Mode'
   ```

3. **异常处理**：  

   - 串口断开自动重连  
   - 数据解析错误日志记录  

> **注意**：所有节点默认波特率为115200，可通过`~baudrate`参数调整  


# T-RTK UM982 使用说明（非 ROS 环境）  
`demo` 目录下分别提供了Python 和 C++ Demo  
* `*_driver_um98x`：读取RTK的NEMA协议，解析 GNGGA 和 GNRMC 指令，在控制台打印经纬高、航向、速度数据；  
* `*_driver_um98x_ntrip`：使用NTRIP请求，获取差分数据，使接收机进入RTK模式，并在控制台打印经纬高、航向、速度数据；  

## 1. 使用方法
请参见相应代码开头的注释说明。