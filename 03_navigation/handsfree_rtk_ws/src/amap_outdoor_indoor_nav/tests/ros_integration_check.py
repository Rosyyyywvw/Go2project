#!/usr/bin/env python3
"""Exercise panel and geo converter on a private localhost ROS master only.

Run explicitly after sourcing ROS Noetic and handsfree_rtk_ws/devel/setup.bash:
    python3 tests/ros_integration_check.py

Starts only a temporary master, the coordinate converter, and the read-only panel.
All synthetic messages use that independent master; no robot drivers are launched.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import urllib.request
import xmlrpc.client

PACKAGE = Path(__file__).resolve().parents[1]
GEO = PACKAGE.parents[1] / 'devel/lib/amap_outdoor_indoor_nav/geo_coordinate_layer'


def free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def wait_until(predicate, description, seconds=15):
    end = time.monotonic() + seconds
    last_error = None
    while time.monotonic() < end:
        try:
            result = predicate()
            if result:
                return result
        except Exception as exc:
            last_error = exc
        time.sleep(.08)
    raise AssertionError(description + (': ' + str(last_error) if last_error else ''))


def gga(quality):
    body = 'GNGGA,123519.00,3113.8240000,N,12128.4220000,E,{},18,0.6,10.0,M,0.0,M,,'.format(quality)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}\r\n'.format(body, checksum)


def main():
    master_port, http_port = free_port(), free_port()
    while master_port == 11311 or http_port in (11311, master_port):
        master_port, http_port = free_port(), free_port()
    temporary = Path(tempfile.mkdtemp(prefix='go2w-amap-isolated-'))
    master_uri = 'http://127.0.0.1:{}'.format(master_port)
    os.environ.update(ROS_MASTER_URI=master_uri, ROS_HOSTNAME='127.0.0.1',
                      ROS_IP='127.0.0.1', ROS_LOG_DIR=str(temporary / 'roslog'))
    os.environ.pop('AMAP_JS_KEY', None)
    os.environ.pop('AMAP_JS_SECURITY_CODE', None)
    assert ':11311' not in os.environ['ROS_MASTER_URI']
    processes, handles = [], []
    checks = []
    report = {'master_port': master_port, 'http_port': http_port,
              'master_uri': master_uri, 'artifacts': str(temporary), 'checks': checks}

    def start(name, command):
        output = open(temporary / (name + '.log'), 'w')
        handles.append(output)
        process = subprocess.Popen(command, env=os.environ.copy(), stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process

    def state():
        with urllib.request.urlopen('http://127.0.0.1:{}/api/state'.format(http_port), timeout=2) as response:
            return json.load(response)

    rospy = None
    try:
        if not (PACKAGE / 'web/index.html').is_file():
            raise RuntimeError('Missing web/index.html; build/check the package first')
        if not GEO.is_file():
            raise RuntimeError('Missing geo_coordinate_layer binary; run catkin_make first')
        start('roscore', ['roscore', '-p', str(master_port)])
        master = xmlrpc.client.ServerProxy(master_uri)
        wait_until(lambda: master.getUri('/amap_isolated_test')[0] == 1, 'isolated master ready')
        import rospy
        from geometry_msgs.msg import Point, PointStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Bool, String
        from visualization_msgs.msg import Marker

        rospy.init_node('amap_isolated_test', disable_signals=True)
        prefix = '/amap_panel_isolation_test'
        topics = {key: prefix + '/' + key for key in ('gnss', 'gcj02', 'raw', 'enu', 'origin', 'geo_status', 'path', 'odom', 'goal', 'reached')}
        geo_args = [str(GEO), '__name:=geo_coordinate_layer_isolated',
                    '_gnss_topic:=' + topics['gnss'], '_nmea_raw_topic:=' + topics['raw'],
                    '_enu_pose_topic:=' + topics['enu'], '_gcj02_fix_topic:=' + topics['gcj02'],
                    '_origin_topic:=' + topics['origin'], '_status_topic:=' + topics['geo_status'],
                    '_origin_mode:=service', '_quality_mode:=rtk_fixed_only',
                    '_require_fix_status:=true', '_nmea_quality_timeout_sec:=1.0',
                    '_publish_gcj02:=true']
        start('geo', geo_args)
        panel_args = ['/usr/bin/python3', str(PACKAGE / 'scripts/amap_panel.py'),
                      '--host', '127.0.0.1', '--port', str(http_port)]
        panel_args.extend('_{}_topic:={}'.format(key, topic) for key, topic in topics.items())
        start('panel', panel_args)
        initial = wait_until(state, 'panel HTTP endpoint ready')
        assert initial['fix'] is None and initial['gcj02'] is None and initial['enu'] is None
        assert initial['topics'] == topics
        with urllib.request.urlopen('http://127.0.0.1:{}/'.format(http_port), timeout=2) as response:
            assert response.status == 200 and b'<html' in response.read().lower()
        checks.append('HTTP HTML/API ready; all custom ROS input topic parameters honored; no fabricated telemetry')

        raw_pub = rospy.Publisher(topics['raw'], String, queue_size=10)
        fix_pub = rospy.Publisher(topics['gnss'], NavSatFix, queue_size=10)
        path_pub = rospy.Publisher(topics['path'], Marker, queue_size=10)
        odom_pub = rospy.Publisher(topics['odom'], Odometry, queue_size=10)
        goal_pub = rospy.Publisher(topics['goal'], PointStamped, queue_size=10)
        reached_pub = rospy.Publisher(topics['reached'], Bool, queue_size=10)
        wait_until(lambda: raw_pub.get_num_connections() >= 2 and fix_pub.get_num_connections() >= 2
                   and path_pub.get_num_connections() >= 1 and odom_pub.get_num_connections() >= 1
                   and goal_pub.get_num_connections() >= 1 and reached_pub.get_num_connections() >= 1,
                   'geo and panel subscribers connected')

        def publish_fixes(quality, seconds=.7):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                raw_pub.publish(String(data=gga(quality)))
                time.sleep(.025)
                fix = NavSatFix()
                fix.header.stamp = rospy.Time.now()
                fix.header.frame_id = 'gps'
                fix.latitude, fix.longitude, fix.altitude = 31.2304, 121.4737, 10.0
                fix.status.status = 2
                fix.position_covariance_type = 2
                fix.position_covariance = [.0001, 0, 0, 0, .0001, 0, 0, 0, .0004]
                fix_pub.publish(fix)
                time.sleep(.025)

        publish_fixes(4)
        current = wait_until(lambda: (value if (value := state())['gcj02'] else None), 'fixed solution converted')
        assert current['quality']['gga_quality'] == 4
        assert current['quality']['satellites'] == 18 and current['quality']['hdop'] == .6
        # Independent, precomputed Shanghai WGS84->GCJ02 reference.
        assert abs(current['gcj02']['latitude'] - 31.22845773757727) < 1e-9, current['gcj02']['latitude']
        assert abs(current['gcj02']['longitude'] - 121.47822305927693) < 1e-9, current['gcj02']['longitude']
        assert current['origin'] is None and current['enu'] is None
        assert len(current['track']) >= 2
        checks.append('GGA checksum+Fixed/18 satellites/HDOP parsed; GCJ02 matches Shanghai numerical reference to 1e-9 degrees')
        checks.append('service origin mode: GCJ02 and trajectory available while ENU/origin remain null')

        publish_fixes(5, .3)
        time.sleep(.2)
        gcj_stamp = state()['gcj02']['stamp']
        publish_fixes(5, .7)
        floated = state()
        assert floated['quality']['gga_quality'] == 5
        assert floated['gcj02']['stamp'] == gcj_stamp
        assert floated['fix']['stamp'] > gcj_stamp
        checks.append('Float GGA=5 stops geo GCJ02 publication despite raw GNSS continuing')
        publish_fixes(4, .5)
        restored = wait_until(lambda: (value if (value := state())['gcj02']['stamp'] > gcj_stamp else None), 'Fixed publication restored')
        assert restored['quality']['gga_quality'] == 4
        checks.append('Fixed GGA=4 resumes GCJ02 publication')

        path = Marker()
        path.header.stamp, path.header.frame_id = rospy.Time.now(), 'world'
        path.ns, path.type, path.action = 'global_path', Marker.LINE_STRIP, Marker.ADD
        path.points = [Point(0, 0, 0), Point(3, 4, 0), Point(5, 8, 0)]
        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.pose.pose.position = Point(.5, 1, 0)
        goal = PointStamped()
        goal.header.frame_id, goal.point = 'world', Point(5, 8, 0)
        path_pub.publish(path)
        odom_pub.publish(odom)
        goal_pub.publish(goal)
        time.sleep(.15)
        reached_pub.publish(Bool(data=True))
        nav = wait_until(lambda: (value['navigation'] if (value := state())['navigation']['reached'] else None), 'FAR navigation read')
        assert nav['path']['frame_id'] == 'world' and len(nav['path']['points']) == 3
        assert nav['path']['points'][1] == {'x': 3., 'y': 4., 'z': 0.}
        assert nav['odometry']['x'] == .5 and nav['odometry']['y'] == 1.
        assert nav['goal']['x'] == 5. and nav['goal']['y'] == 8.
        assert nav['reached']['value'] is True
        checks.append('FAR LINE_STRIP, Odometry, PointStamped goal and reached Bool consumed through actual ROS subscriptions')

        path.action = Marker.DELETE
        path_pub.publish(path)
        wait_until(lambda: state()['navigation']['path'] is None, 'deleted FAR path cleared')
        checks.append('FAR Marker.DELETE clears obsolete path')

        code, _, graph = master.getSystemState('/amap_isolated_test')
        assert code == 1
        publishers = graph[0]
        panel_topics = [topic for topic, nodes in publishers if '/amap_panel' in nodes]
        assert set(panel_topics) <= {'/rosout'}, panel_topics
        assert not any(topic in ('/cmd_vel', '/goal_point', '/way_point') for topic, nodes in publishers)
        checks.append('ROS graph confirms panel publishes no control/goal topics; its only optional publication is /rosout')
        report['result'] = 'PASS'
    except BaseException as exc:
        report['result'] = 'FAIL'
        report['error'] = '{}: {}'.format(type(exc).__name__, exc)
        raise
    finally:
        if rospy is not None:
            rospy.signal_shutdown('Isolated panel integration test completed')
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        for handle in handles:
            handle.close()
        report['processes_cleaned'] = all(process.poll() is not None for _, process in processes)
        (temporary / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
