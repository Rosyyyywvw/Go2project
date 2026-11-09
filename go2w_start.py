#!/usr/bin/env python3
"""Start the already configured Go2W ROS1 navigation stack on Ubuntu.

Copy this file to the robot. Run: python3 /path/to/go2w_start.py start
Only this script's child process groups are stopped by its stop command.
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import xmlrpc.client


HOME = Path.home()
STATE_DIR = HOME / ".local" / "state" / "go2w_start"
STATE_FILE = STATE_DIR / "processes.json"
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
SCRIPT_VERSION = "2026-10-21-local-only-16"


def first_file(*paths):
    return next((path for path in paths if path.is_file()), None)


def require(path, description):
    if path is None or not path.is_file():
        raise RuntimeError(f"找不到 {description}：{path or '已检查的候选路径均不存在'}")
    return path


def shell_line(setups, argv):
    parts = [f"source {shlex.quote(str(path))}{' --extend' if extend else ''}"
             for path, extend in setups]
    parts.append("exec " + shlex.join([str(item) for item in argv]))
    return " && ".join(parts)


def ros_call(setups, argv, timeout=12, output=True):
    result = subprocess.run(
        ["bash", "-lc", shell_line(setups, argv)],
        stdout=subprocess.PIPE if output else subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )
    return result.returncode, result.stdout.strip() if output else "", result.stderr.strip()


def topic_has_message(setups, topic, seconds=12):
    try:
        code, _, _ = ros_call(setups, ["rostopic", "echo", "-n", "1", topic],
                              timeout=seconds, output=False)
        return code == 0
    except subprocess.TimeoutExpired:
        return False


def master_alive():
    try:
        code, _, _ = ros_call([(ROS_SETUP, False)], ["rosparam", "list"], timeout=3)
        return code == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def publish_robot_description(model_path):
    description = model_path.read_text(encoding="utf-8")
    root = ET.fromstring(description)
    if root.tag != "robot" or root.get("name") != "go2w_description":
        raise RuntimeError(f"车体模型不是 Go2W URDF：{model_path}")
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    master = xmlrpc.client.ServerProxy(master_uri)
    code, message, _ = master.setParam("/go2w_start", "/robot_description", description)
    if code != 1:
        raise RuntimeError(f"写入 /robot_description 失败：{message}")
    print(f"/robot_description: 已加载 {model_path}")


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def load_state():
    if not STATE_FILE.is_file():
        return {"processes": {}}
    return json.loads(STATE_FILE.read_text(encoding="utf-8"))


def save_state(state):
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    temporary = STATE_FILE.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(STATE_FILE)


def tail(path, lines=12):
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-lines:])
    except OSError:
        return "日志不可读"


def spawn(state, label, setups, argv, extra_env=None):
    old = state["processes"].get(label)
    if old and pid_alive(old["pid"]):
        print(f"{label}: 已由本脚本启动，PID {old['pid']}；不会重新加载本次选中的 launch")
        return
    log_path = Path(state["log_dir"]) / f"{label}.log"
    command = shell_line(setups, argv)
    with log_path.open("w", encoding="utf-8") as log:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        if extra_env:
            env.update(extra_env)
        log.write(f"启动命令：{command}\n")
        log.flush()
        process = subprocess.Popen(
            ["bash", "-lc", command],
            cwd=HOME,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=env,
        )
    state["processes"][label] = {"pid": process.pid, "log": str(log_path)}
    save_state(state)
    print(f"{label}: PID {process.pid}，命令 {command}，日志 {log_path}")


def launch_arg_defaults(path):
    root = ET.parse(path).getroot()
    return {item.get("name"): item.get("default") for item in root.findall("arg")}


def wait_for(state, label, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print(f"{label}: 已收到数据")
            return
        item = state["processes"].get(label)
        if item and not pid_alive(item["pid"]):
            raise RuntimeError(f"{label} 已退出。日志末尾：\n{tail(item['log'])}")
        time.sleep(1)
    item = state["processes"].get(label)
    suffix = f"\n日志末尾：\n{tail(item['log'])}" if item else ""
    raise RuntimeError(f"等待 {label} 超时。{suffix}")


def package_dir(setups, package):
    code, out, err = ros_call(setups, ["rospack", "find", package])
    if code != 0:
        raise RuntimeError(f"ROS 找不到 {package}：{err}")
    return Path(out)


def node_alive(setups, name):
    try:
        code, nodes, _ = ros_call(setups, ["rosnode", "list"], timeout=4)
        return code == 0 and name in nodes.splitlines()
    except subprocess.TimeoutExpired:
        return False


def vehicle_tf_available(setups):
    predicate = ('any(t.header.frame_id == "world" and '
                 't.child_frame_id == "vehicle" for t in m.transforms)')
    try:
        code, _, _ = ros_call(setups, ["rostopic", "echo", "-n", "1",
                                     "--filter", predicate, "/tf"],
                              timeout=3, output=False)
        return code == 0
    except subprocess.TimeoutExpired:
        return False


def checked_speed(setups, param, limit):
    code, value, err = ros_call(setups, ["rosparam", "get", param])
    if code != 0:
        raise RuntimeError(f"缺少速度参数 {param}：{err}")
    try:
        speed = float(value)
    except ValueError as exc:
        raise RuntimeError(f"速度参数 {param} 不是数值：{value}") from exc
    if speed < 0 or speed > limit:
        raise RuntimeError(f"{param}={speed} 超出本脚本允许的 0～{limit} m/s；底盘订阅未启动")
    print(f"{param}={speed} m/s")


def cmd_subscribers(setups):
    try:
        code, info, _ = ros_call(setups, ["rostopic", "info", "/cmd_vel"], timeout=4)
        if code != 0 or "Subscribers:" not in info:
            return []
        section = info.split("Subscribers:", 1)[1]
        names = [line.strip().split()[1] for line in section.splitlines()
                 if line.strip().startswith("* /")]
        # rostopic echo -n 1 leaves a short-lived (sometimes stale) ROS master entry.
        return [name for name in names if not name.startswith("/rostopic_")]
    except subprocess.TimeoutExpired:
        return []


def configured_paths():
    autonomous = require(
        Path(__file__).resolve().parent /
        "03_navigation/local/autonomous_exploration_development_environment/devel/setup.bash",
        "local 工作区 setup.bash")
    far = require(first_file(
        HOME / "farplanner_ws/src/far_planner-melodic-noetic/devel/setup.bash",
        HOME / "Go2W_Project/03_navigation/farplanner_ws/src/far_planner-melodic-noetic/devel/setup.bash",
    ), "FAR 工作区 setup.bash")
    unitree = require(HOME / "unitree_ws/devel/setup.bash", "Unitree 模型工作区 setup.bash")
    model = require(HOME / "unitree_ws/src/unitree_ros/robots/go2w_description/urdf/go2w_description.urdf",
                    "Go2W URDF")
    sdk_roots = [HOME / "Go2W_Project/04_unitree/unitree_sdk2_for_nav",
                 HOME / "unitree_sdk2_for_nav"]
    imu_script = require(first_file(HOME / "unitree_sdk2_for_nav/go2w_lowstate_imu.py",
                                    HOME / "Go2W_Project/04_unitree/unitree_sdk2_for_nav/go2w_lowstate_imu.py"),
                         "go2w_lowstate_imu.py")
    sdk_root = next((root for root in sdk_roots if (root / "build/bin/go2_move_sub_twist").is_file()), None)
    if sdk_root is None:
        raise RuntimeError("找不到已编译的 go2_move_sub_twist（检查 Unitree SDK 工作区）")
    return {
        "driver": require(HOME / "robosense_ws/devel/setup.bash", "RS32 驱动工作区"),
        "converter": require(HOME / "rstovelody_ws/devel/setup.bash", "RS32 点云转换工作区"),
        "lio": require(HOME / "Super-LIO/devel/setup.bash", "Super-LIO 工作区"),
        "autonomous": autonomous,
        "far": far,
        "unitree": unitree,
        "model": model,
        "imu_script": imu_script,
        "sdk_root": sdk_root,
    }


def start(args):
    require(ROS_SETUP, "ROS Noetic")
    paths = configured_paths()  # Fail before launching anything if a workspace is absent.
    core_env = [(ROS_SETUP, False)]
    driver_env = [(ROS_SETUP, False), (paths["driver"], False)]
    converter_env = [(ROS_SETUP, False), (paths["converter"], False)]
    lio_env = [(ROS_SETUP, False), (paths["lio"], False)]
    auto_env = [(ROS_SETUP, False), (paths["autonomous"], False)]
    far_env = auto_env + [(paths["unitree"], True), (paths["far"], True)]

    # Confirm installed launch files, preserving the working configuration.
    driver_launch = package_dir(driver_env, "rslidar_sdk") / "launch/start.launch"
    require(driver_launch, "RS32 start.launch")
    lio_package = package_dir(lio_env, "super_lio")
    lio_launch = require(lio_package / "launch/M2DGR.launch", "Super-LIO M2DGR.launch")
    local_workspace = paths["autonomous"].parent.parent
    local_package = package_dir(auto_env, "local_planner")
    vehicle_package = package_dir(auto_env, "vehicle_simulator")
    for package in (local_package, vehicle_package):
        if (local_workspace / "src").resolve() not in package.resolve().parents:
            raise RuntimeError(f"导航包没有指向 local 工作区：{package}")
    local_launch = require(vehicle_package / "launch/system_real_robot.launch",
                           "local 工作区 system_real_robot.launch")
    local_rviz_config = require(vehicle_package / "rviz/vehicle_simulator.rviz",
                                "local 工作区 RViz 配置")
    for package, binary in (("local_planner", "localPlanner"),
                            ("local_planner", "pathFollower"),
                            ("terrain_analysis", "terrainAnalysis"),
                            ("terrain_analysis_ext", "terrainAnalysisExt")):
        require(local_workspace / "devel/lib" / package / binary,
                f"local 工作区已编译的 {binary}")
    print(f"local_planner: 选中的 launch 为 {local_launch}")
    local_defaults = launch_arg_defaults(require(
        local_package / "launch/local_planner.launch", "local 工作区局部规划 launch"))
    system_defaults = launch_arg_defaults(local_launch)
    for flag in ("publishSensorTf", "publishCameraTf"):
        local_defaults[flag] = system_defaults[flag]
    for flag in ("publishSensorTf", "publishCameraTf"):
        if flag in local_defaults:
            print(f"local_planner: {flag} 启动值为 {local_defaults[flag]}")
    far_launch = require(package_dir(far_env, "far_planner") / "launch/far_planner_go2w.launch",
                         "far_planner_go2w.launch")

    state = load_state()
    if not state.get("log_dir") or not any(pid_alive(v["pid"]) for v in state["processes"].values()):
        run_id = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        state = {"log_dir": str(STATE_DIR / "logs" / run_id), "processes": {}}
        Path(state["log_dir"]).mkdir(parents=True, exist_ok=True)
        save_state(state)

    if not master_alive():
        spawn(state, "roscore", core_env, ["roscore"])
        wait_for(state, "roscore", master_alive, 20)
    else:
        print("roscore: 已在运行")

    publish_robot_description(paths["model"])

    spawn(state, "rs32", driver_env, ["roslaunch", "rslidar_sdk", driver_launch.name,
                                     "rviz:=false"])
    wait_for(state, "rs32", lambda: topic_has_message(core_env, "/rslidar_points", 3), 60)

    spawn(state, "point_convert", converter_env,
          ["rosrun", "rs_to_velodyne", "rs_to_velodyne", "XYZIRT", "XYZIRT"])
    wait_for(state, "point_convert", lambda: topic_has_message(core_env, "/velodyne_points", 3), 40)

    spawn(state, "imu", core_env, ["python3", paths["imu_script"], args.iface])
    wait_for(state, "imu", lambda: topic_has_message(core_env, "/imu_data_highrate", 3), 30)

    # M2DGR.launch is the user's working LIO launch. Navigation opens the two
    # dedicated RViz windows below, so the LIO launch must not open a third one.
    lio_args = ["roslaunch", "super_lio", lio_launch.name]
    if 'name="rviz"' in lio_launch.read_text(errors="replace"):
        lio_args.append("rviz:=false")
    spawn(state, "lio", lio_env, lio_args)
    wait_for(state, "lio", lambda: topic_has_message(core_env, "/lio/odom", 3), 90)
    wait_for(state, "lio", lambda: topic_has_message(core_env, "/lio/cloud_world", 3), 30)

    use_terrain = getattr(args, "terrain_mode", "terrain") == "terrain"
    print(f"localPlanner: useTerrainAnalysis={str(use_terrain).lower()}（使用 {'/terrain_map' if use_terrain else '/lio/cloud_world'}），twoWayDrive=false（禁止无保护倒车）")
    print("localPlanner 参数组：使用 local 工作区 launch 的原有规划和地形参数")
    local_launch_args = ["roslaunch", str(local_launch)]
    if not use_terrain:
        local_launch_args.append("useTerrainAnalysis:=false")
    spawn(state, "local_planner", auto_env,
          local_launch_args)
    wait_for(state, "local_planner",
             lambda: node_alive(core_env, "/localPlanner") and node_alive(core_env, "/pathFollower"), 30)
    if args.start_stopped:
        # This follower supports /stop but has no initialSafetyStop parameter.
        # Apply the stop after its subscriber exists, before starting the drive.
        code, _, err = ros_call(core_env,
                               ["rostopic", "pub", "-1", "/stop", "std_msgs/Int8", "data: 2"])
        if code != 0:
            raise RuntimeError(f"设置局部规划器 /stop=2 失败：{err}")
        print("localPlanner: 已发布 /stop=2，显式解除后才执行目标")
    if os.environ.get("DISPLAY"):
        rviz_env = {"__GLX_VENDOR_LIBRARY_NAME": "mesa", "LIBGL_ALWAYS_SOFTWARE": "1"}
        # The local view uses FAR's GoalPointTool plugin, so inherit both the
        # autonomous and FAR workspaces even though its displays are localPlanner-specific.
        spawn(state, "local_rviz", far_env,
              ["rosrun", "rviz", "rviz", "-d", local_rviz_config,
               "__name:=local_planner_rviz", "/state_estimation:=/lio/odom"], extra_env=rviz_env)
        wait_for(state, "local_rviz",
                 lambda: node_alive(core_env, "/local_planner_rviz"), 30)
    else:
        print("local_rviz: 未设置 DISPLAY，跳过图形界面")
    for node_name, flag in (("vehicleTransPublisher", "publishSensorTf"),
                            ("sensorTransPublisher", "publishCameraTf")):
        running = node_alive(core_env, f"/{node_name}")
        default = local_defaults.get(flag, "未声明")
        print(f"TF 节点 /{node_name}: {'运行中' if running else '未运行'}；{flag} 启动值 {default}")

    if local_defaults.get("publishSensorTf", "false").lower() != "false":
        raise RuntimeError("publishSensorTf 已启用，不能同时发布另一条 vehicle 父变换")
    vehicle_tf_script = require(Path(__file__).resolve().with_name("vehicle_tf_from_lio.py"),
                                "world→vehicle TF 节点")
    spawn(state, "vehicle_tf", core_env, ["python3", vehicle_tf_script])
    wait_for(state, "vehicle_tf", lambda: vehicle_tf_available(core_env), 20)

    spawn(state, "far_planner", far_env,
          ["roslaunch", "far_planner", far_launch.name,
           "start_terrain:=false",
           "start_rviz:=true" if os.environ.get("DISPLAY") else "start_rviz:=false"])
    wait_for(state, "far_planner", lambda: node_alive(core_env, "/far_planner"), 60)
    # FAR needs terrain maps even when the local planner uses raw clouds.
    for topic in ("/terrain_map", "/terrain_map_ext"):
        wait_for(state, "local_planner", lambda topic=topic: topic_has_message(core_env, topic, 5), 30)
    wait_for(state, "local_planner", lambda: topic_has_message(core_env, "/cmd_vel", 3), 45)

    code, cmd_type, err = ros_call(core_env, ["rostopic", "type", "/cmd_vel"])
    if code != 0:
        raise RuntimeError(f"无法确定 /cmd_vel 类型：{err}")
    binary_name = {"geometry_msgs/Twist": "go2_move_sub_twist",
                   "geometry_msgs/TwistStamped": "go2_move_sub2"}.get(cmd_type)
    if binary_name is None:
        raise RuntimeError(f"/cmd_vel 类型 {cmd_type} 不受 Unitree 订阅程序支持")
    print(f"/cmd_vel 类型：{cmd_type}；对应订阅程序：{binary_name}")

    # Validate against the user's speed limits in the selected Go2W launch.
    for parameter, limit in (
            ("/localPlanner/maxSpeed", float(local_defaults["maxSpeed"])),
            ("/localPlanner/autonomySpeed", float(local_defaults["autonomySpeed"])),
            ("/pathFollower/maxSpeed", float(local_defaults["maxSpeed"])),
            ("/pathFollower/autonomySpeed", float(local_defaults["autonomySpeed"]))):
        checked_speed(core_env, parameter, limit)

    if args.no_drive:
        print("底盘订阅未启动（--no-drive）。其余节点已启动。")
        return
    subscribers = cmd_subscribers(core_env)
    if subscribers:
        print(f"/cmd_vel 已有非临时订阅者，未重复启动底盘程序：{', '.join(subscribers)}")
        return
    binary = require(paths["sdk_root"] / "build/bin" / binary_name, binary_name)
    sdk_lib = paths["sdk_root"] / "thirdparty/lib/aarch64"
    extra_env = {"LD_LIBRARY_PATH": f"{sdk_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"}
    spawn(state, "drive", core_env, [binary, args.iface], extra_env=extra_env)
    wait_for(state, "drive", lambda: bool(cmd_subscribers(core_env)), 15)
    print("启动完成。FAR RViz 和 localPlanner RViz 均已加载；确认 F1 控制状态。")


def stop():
    state = load_state()
    for label, item in reversed(list(state["processes"].items())):
        pid = item["pid"]
        if pid_alive(pid):
            try:
                os.killpg(pid, signal.SIGINT)
                print(f"已请求停止 {label} (PID {pid})")
            except ProcessLookupError:
                pass
    time.sleep(2)
    for label, item in reversed(list(state["processes"].items())):
        if pid_alive(item["pid"]):
            try:
                os.killpg(item["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
    if STATE_FILE.exists():
        STATE_FILE.unlink()


def status():
    state = load_state()
    if not state["processes"]:
        print("本脚本没有记录正在运行的节点。")
        return
    for label, item in state["processes"].items():
        print(f"{label}: {'运行中' if pid_alive(item['pid']) else '已退出'}，PID {item['pid']}，日志 {item['log']}")


def main():
    parser = argparse.ArgumentParser(description="Go2W RS32 + LIO + localPlanner + FAR 一键启动")
    parser.add_argument("action", choices=["start", "status", "stop"])
    parser.add_argument("--iface", default="eth0", help="Go2W DDS 网卡，默认 eth0")
    parser.add_argument("--terrain-mode", choices=["terrain", "raw"], default="terrain",
                        help="局部规划输入：默认 terrain 使用 /terrain_map；raw 使用原始 LIO 点云")
    parser.add_argument("--no-drive", action="store_true", help="只启动感知和规划，不启动底盘订阅")
    parser.add_argument("--start-stopped", action="store_true", help="启动底盘订阅前设置 follower 的 /stop=2，显式解除后才执行目标")
    args = parser.parse_args()
    try:
        if args.action == "start":
            print(f"go2w_start.py 版本：{SCRIPT_VERSION}")
            start(args)
        elif args.action == "stop":
            stop()
        else:
            status()
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        script = Path(__file__).resolve()
        print(f"查看日志：python3 {script} status；停止本脚本启动的节点：python3 {script} stop",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

