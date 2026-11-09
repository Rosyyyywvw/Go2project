[Chinese Simplified](README.md)|[English]  

**[Click to view online tutorial](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvEwyKv4gQ1mDA)**  

# T-RTK UM982 Usage Guide (ROS Environment)  

## 1. Installation  

### 1.1 Install Dependencies  
Use the following commands to install the serial communication dependency:  

```sh
pip install pyserial   # For Python 2  
pip3 install pyserial  # For Python 3  
```

### 1.2 Build  
The handsfree_rtk package consists of a Python driver script file, a USB mount point, udev rules, and a launch file  

First, create a workspace (the workspace name can be customized here, but the default is catkin_ws)  

```bash
cd & mkdir -p catkin_ws/src
```

Download the handsfree_rtk.zip in our online documentation and extract it into ~/catkin_ws/src  
Open the terminal in the unzipped handsfree_rtk folder and run the following command to install it.  

```bash
cd ~/catkin_ws/src/handsfree_rtk/
bash auto_install.sh
```

## 2. Usage
You can use the following command to set the ROS of handsfree_rtk to the .bashrc file for later use, "~/catkin_ws/" is the workspace, please modify it according to the actual situation.  

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 2.1 handsfree_rtk.launch  

- **Features**:  
The RTK module driver parses the GNGGA, GNRMC data .  
- **Serial port name**:  
    `/dev/HFRobotRTK` (modifiable via `~port` parameter)  
- **Published Topics**:  
  - `handsfree/rtk/raw` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)) — Raw NMEA protocol data directly output from the GNSS receiver.  
  - `handsfree/rtk/gnss` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)) — Parsed GNSS positioning data, including fix status, latitude, longitude, altitude, and covariance matrix.  
  - `handsfree/rtk/speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — Ground speed parsed from NMEA data, in **m/s**. 
  - `handsfree/rtk/cog` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — Course Over Ground (COG), representing the **movement direction** relative to true north, measured clockwise (0° = North, 90° = East, 180° = South, 270° = West), in **degrees (°)**. 
  - `handsfree/rtk/heading` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — True Heading, representing the **device’s orientation** (typically the dual-antenna baseline direction), independent of movement direction, in **degrees (°)**. 
- **Launch Command**:  
    ```bash
    roslaunch handsfree_rtk handsfree_rtk.launch
    ```

### 2.2 handsfree_rtk_ntrip.launch  

- **Features**:  
NTRIP client driver to realize differential data transmission between base station and mobile station. Automatically upload the GGA data of the mobile station to the NTRIP server, and send the received RTCM differential data to the mobile station.  

- **Serial port name**:  
    `/dev/HFRobotRTK` (modifiable via `~port` parameter)  
- **Receiving/Sending**:  
    - Send `GPGGA 1` command to the serial port (trigger the mobile station to output GGA data)  
    - Send GGA observations to the NTRIP server  
    - Received from NTRIP server: RTCM3.x differential data stream  
- **Published Topics**:  
  - `handsfree/rtk/raw` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)) — Raw NMEA protocol data directly output from the GNSS receiver.  
  - `handsfree/rtk/gnss` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)) — Parsed GNSS positioning data, including fix status, latitude, longitude, altitude, and covariance matrix.  
  - `handsfree/rtk/speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — Ground speed parsed from NMEA data, in **m/s**. 
  - `handsfree/rtk/cog` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — Course Over Ground (COG), representing the **movement direction** relative to true north, measured clockwise (0° = North, 90° = East, 180° = South, 270° = West), in **degrees (°)**. 
  - `handsfree/rtk/heading` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) — True Heading, representing the **device’s orientation** (typically the dual-antenna baseline direction), independent of movement direction, in **degrees (°)**. 
- **Launch Command**:  
    ```bash
    roslaunch handsfree_rtk handsfree_rtk_ntrip.launch
    ```
- **Key Parameters**:  
    ```xml
        <!-- NTRIP Server Configuration -->
        <arg name="ntrip_server" default="120.253.239.161"/>
        <arg name="ntrip_port" default="8002"/>
        <arg name="ntrip_username" default="$(optenv NTRIP_USERNAME)"/>
        <arg name="ntrip_password" default="$(optenv NTRIP_PASSWORD)"/>
        <arg name="ntrip_mountpoint" default="$(optenv NTRIP_MOUNTPOINT)"/>
    ```

### 2.3 NTRIP CORS Account Configuration  

These parameters are used to configure the NTRIP client in the RTK differential location service, connecting to the CORS (Continuous Operating Reference Station System) service provided by the state or commercial organization. Here's a detailed explanation of each parameter:  

| The name of the parameter | Example values | illustrate |
| --- | --- | --- |
| ntrip_server | 120.253.239.161 | The IP address or domain name of the NTRIP server, pointing to the CORS central server that provides the differential correction data |
| ntrip_port | 8002 | TCP port number used by NTRIP (common port: 2101/8001/8002) |
| ntrip_username | (环境变量 NTRIP_USERNAME) | Authentication username (provided by the CORS service provider to identify the user and perform billing) |
| ntrip_password | (环境变量 NTRIP_PASSWORD) | Authentication password (paired with username for service access authentication) |
| ntrip_mountpoint | RTCM33_GRCE | Mount point name (specifies which type/region of differential data stream to get) |

---

## 3. General features  

1. **GNGGA statement state mapping**:  
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

2. **GNRMC statement state mapping**  

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

2.  **Exception Handling**:  
    * The serial port is automatically reconnected when it is disconnected  
    * Data parsing error logging  

> **Note**: The default baud rate for all nodes is 115200, which can be adjusted via the `~baudrate` parameter  


# T-RTK UM982 Usage Guide (Non-ROS Environment)  

The `demo` directory provides Python and C++ demos:  

* **`*_driver_um98x*`**: Reads RTK NMEA messages, parses **GNGGA** and **GNRMC** sentences, and prints latitude, longitude, altitude, heading, and speed data to the console.  
* **`*_driver_um98x_ntrip*`**: Uses an **NTRIP** request to obtain differential correction data, enables RTK mode on the receiver, and prints latitude, longitude, altitude, heading, and speed data to the console.  

## 1. Usage  

Please refer to the comments at the beginning of the source code for detailed instructions.  
