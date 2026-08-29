#!/usr/bin/env python3
"""Launch and verify the complete headless Gazebo stack with cleanup and logs."""

import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

import rosgraph
import rosnode
import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


EXPECTED_NODES = {
    "/gazebo",
    "/emotion_bot/adapter",
    "/emotion_bot/expression_mapper",
    "/emotion_bot/safety_bridge",
    "/lite3_sim_runner",
}
EXPECTED_CONTROLLERS = {
    "joint_states_controller",
    "FL_HipX", "FL_HipY", "FL_Knee",
    "FR_HipX", "FR_HipY", "FR_Knee",
    "HL_HipX", "HL_HipY", "HL_Knee",
    "HR_HipX", "HR_HipY", "HR_Knee",
}
MIN_STANDING_HEIGHT_M = 0.18
MAX_STANDING_HEIGHT_M = 0.40


def wait_wall(predicate, description, timeout=120.0, interval=0.25):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            value = predicate()
            if value:
                return value
        except Exception as exc:
            last_error = exc
        time.sleep(interval)
    raise RuntimeError("timed out waiting for %s (last error: %r)" % (description, last_error))


def wait_message(topic, msg_type, predicate=lambda _msg: True, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            message = rospy.wait_for_message(topic, msg_type, timeout=0.75)
        except rospy.ROSException:
            continue
        if predicate(message):
            return message
    raise RuntimeError("timed out waiting for message on %s" % topic)


def link_sample(get_link):
    response = get_link("lite3_gazebo::TORSO", "world")
    if not response.success:
        raise RuntimeError(response.status_message)
    pose = response.link_state.pose.position
    twist = response.link_state.twist
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "vx": twist.linear.x,
        "vy": twist.linear.y,
        "vz": twist.linear.z,
        "yaw_rate": twist.angular.z,
    }


def planar_distance(first, second):
    return math.hypot(second["x"] - first["x"], second["y"] - first["y"])


def assert_standing(sample, phase):
    if not MIN_STANDING_HEIGHT_M <= sample["z"] <= MAX_STANDING_HEIGHT_M:
        raise RuntimeError(
            "Lite3 torso height %.4f m outside standing range [%.2f, %.2f] during %s"
            % (sample["z"], MIN_STANDING_HEIGHT_M, MAX_STANDING_HEIGHT_M, phase)
        )


def wait_for_planar_stop(get_link, threshold=0.04, timeout=45.0, stable_samples=3):
    deadline = time.monotonic() + timeout
    stable = 0
    last_sample = None
    last_speed = float("inf")
    while time.monotonic() < deadline:
        last_sample = link_sample(get_link)
        last_speed = math.hypot(last_sample["vx"], last_sample["vy"])
        if last_speed <= threshold:
            stable += 1
            if stable >= stable_samples:
                return last_sample, last_speed
        else:
            stable = 0
        time.sleep(0.4)
    raise RuntimeError("Gazebo body did not settle below %.3f m/s (last %.4f m/s)" % (threshold, last_speed))


def publish_for(publisher, message, ros_duration, rate_hz=10.0):
    end = rospy.Time.now() + rospy.Duration(ros_duration)
    wall_deadline = time.monotonic() + max(10.0, ros_duration * 30.0)
    while not rospy.is_shutdown() and rospy.Time.now() < end and time.monotonic() < wall_deadline:
        publisher.publish(message)
        time.sleep(1.0 / rate_hz)
    if rospy.Time.now() < end:
        raise RuntimeError("simulated time stalled while publishing command")


def sim_process_udp_sockets():
    udp_endpoints = {}
    for table in ("/proc/net/udp",):
        try:
            with open(table, "r", encoding="ascii") as stream:
                for line in list(stream)[1:]:
                    columns = line.split()
                    if len(columns) >= 10:
                        local_hex, remote_hex = columns[1], columns[2]
                        local_address, local_port = local_hex.split(":")
                        remote_address, remote_port = remote_hex.split(":")
                        udp_endpoints[columns[9]] = {
                            "local_ip": socket.inet_ntoa(struct.pack("<I", int(local_address, 16))),
                            "local_port": int(local_port, 16),
                            "remote_ip": socket.inet_ntoa(struct.pack("<I", int(remote_address, 16))),
                            "remote_port": int(remote_port, 16),
                        }
        except OSError:
            pass
    matches = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % name, "rb") as stream:
                command = stream.read().replace(b"\0", b" ").decode("utf-8", "replace")
            if "example_lite3_sim" not in command:
                continue
            for fd_name in os.listdir("/proc/%s/fd" % name):
                target = os.readlink("/proc/%s/fd/%s" % (name, fd_name))
                if target.startswith("socket:[") and target[8:-1] in udp_endpoints:
                    item = {"pid": int(name), "fd": fd_name, "socket": target}
                    item.update(udp_endpoints[target[8:-1]])
                    matches.append(item)
        except (OSError, PermissionError):
            continue
    return matches


def main():
    log_handle = tempfile.NamedTemporaryFile(
        prefix="emotion_bot_gazebo_e2e_", suffix=".log", delete=False, mode="w+"
    )
    log_path = log_handle.name
    command = [
        "roslaunch", "emotion_bot_ros", "integrated_sim.launch",
        "gui:=false", "headless:=true", "motion_enabled:=false",
    ]
    launch = subprocess.Popen(
        command,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    results = {"log": log_path}
    set_motion = None
    try:
        def master_ready():
            if launch.poll() is not None:
                raise RuntimeError("roslaunch exited with %d" % launch.returncode)
            return rosgraph.Master("/emotion_bot_e2e_probe").getPid()

        wait_wall(master_ready, "ROS master", timeout=30.0)
        rospy.init_node("emotion_bot_gazebo_e2e", anonymous=True, disable_signals=True)

        wait_wall(
            lambda: EXPECTED_NODES.issubset(set(rosnode.get_node_names())),
            "integrated nodes",
            timeout=120.0,
        )
        rospy.wait_for_service("/lite3_gazebo/controller_manager/list_controllers", timeout=90.0)
        list_controllers = rospy.ServiceProxy(
            "/lite3_gazebo/controller_manager/list_controllers", ListControllers
        )
        running = wait_wall(
            lambda: {
                controller.name
                for controller in list_controllers().controller
                if controller.state == "running"
            },
            "running controllers",
            timeout=60.0,
        )
        if not EXPECTED_CONTROLLERS.issubset(running):
            raise RuntimeError("controllers missing: %s" % sorted(EXPECTED_CONTROLLERS - running))
        results["controllers"] = sorted(running)

        joint = wait_message("/lite3_gazebo/joint_states", JointState, timeout=90.0)
        if len(joint.name) < 12:
            raise RuntimeError("incomplete Lite3 joint state")
        results["joint_count"] = len(joint.name)
        wait_message("/emotion_bot/sim_controller_ready", Bool, lambda msg: msg.data, timeout=120.0)

        rospy.wait_for_service("/gazebo/get_link_state", timeout=30.0)
        get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        rospy.wait_for_service("/emotion_bot/set_motion_enabled", timeout=20.0)
        set_motion = rospy.ServiceProxy("/emotion_bot/set_motion_enabled", SetBool)
        input_pub = rospy.Publisher("/emotion_bot/input", String, queue_size=10)
        direct_pub = rospy.Publisher("/emotion_bot/expression_cmd", Twist, queue_size=10)
        manual_pub = rospy.Publisher("/emotion_bot/manual_joy", Joy, queue_size=10)
        responses = []
        response_sub = rospy.Subscriber(
            "/emotion_bot/response", String, lambda message: responses.append(message.data), queue_size=10
        )
        rospy.sleep(0.5)

        initial_state = json.loads(wait_message("/emotion_bot/state", String).data)
        input_pub.publish(String(data="event:joy"))
        changed = json.loads(
            wait_message(
                "/emotion_bot/state", String,
                lambda msg: json.loads(msg.data)["sequence"] > initial_state["sequence"],
            ).data
        )
        if changed["emotion"] != "joy":
            raise RuntimeError("expected joy, got %s" % changed["emotion"])
        wait_wall(lambda: responses[-1] if responses else None, "emotion response", timeout=5.0)
        response = responses[-1]
        results["emotion"] = changed
        results["response"] = response

        before = link_sample(get_link)
        assert_standing(before, "pre-expression")
        if not set_motion(True).success:
            raise RuntimeError("motion enable service failed")
        safe = wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: abs(msg.linear.x) > 0.01,
            timeout=30.0,
        )
        if abs(safe.linear.x) > 0.100001 or abs(safe.linear.y) > 0.050001 or abs(safe.angular.z) > 0.100001:
            raise RuntimeError("unsafe expression command observed")

        peak_distance = 0.0
        peak_speed = 0.0
        moved_sample = before
        movement_deadline = time.monotonic() + 45.0
        while time.monotonic() < movement_deadline:
            sample = link_sample(get_link)
            distance = planar_distance(before, sample)
            speed = math.hypot(sample["vx"], sample["vy"])
            if distance > peak_distance:
                peak_distance = distance
                moved_sample = sample
            peak_speed = max(peak_speed, speed)
            if peak_distance >= 0.015:
                break
            time.sleep(0.4)
        if peak_distance < 0.015:
            raise RuntimeError("Gazebo body did not measurably move")
        assert_standing(moved_sample, "emotion movement")
        results["emotion_motion"] = {
            "before": before,
            "after": moved_sample,
            "planar_displacement_m": peak_distance,
            "peak_planar_speed_mps": peak_speed,
        }

        set_motion(False)
        wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0,
            timeout=10.0,
        )
        stopped, stopped_speed = wait_for_planar_stop(get_link)
        assert_standing(stopped, "disabled stop")
        results["disabled_stop"] = {"sample": stopped, "planar_speed_mps": stopped_speed}

        # Remove the mapper, drive the bridge directly, then stop publishing to
        # prove its independent command watchdog reaches zero.
        rosnode.kill_nodes(["/emotion_bot/expression_mapper"])
        set_motion(True)
        direct = Twist()
        direct.linear.x = 0.04
        # Keep the stimulus alive past the configured stand-to-locomotion dwell;
        # then stop publishing so the independent bridge watchdog can expire it.
        publish_for(direct_pub, direct, 2.2)
        wait_message("/emotion_bot/safe_cmd", Twist, lambda msg: msg.linear.x > 0.01, timeout=10.0)
        stale_zero = wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0,
            timeout=15.0,
        )
        status = json.loads(
            wait_message(
                "/emotion_bot/status", String,
                lambda msg: json.loads(msg.data)["stale"],
                timeout=10.0,
            ).data
        )
        results["watchdog"] = {"zero": stale_zero.linear.x == 0.0, "status": status}

        # Reproduce the keyboard B/X/axis sequence on the arbitrated manual
        # topic; manual has priority over an emotion command.
        manual_before = link_sample(get_link)
        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 11
        joy.buttons[1] = 1
        manual_pub.publish(joy)
        rospy.sleep(1.6)
        joy.buttons = [0] * 11
        joy.buttons[2] = 1
        manual_pub.publish(joy)
        rospy.sleep(0.5)
        joy.buttons = [0] * 11
        joy.axes[4] = 0.4
        publish_for(manual_pub, joy, 2.0)
        manual_status = json.loads(
            wait_message(
                "/emotion_bot/status", String,
                lambda msg: json.loads(msg.data)["selected_source"] == "manual",
                timeout=10.0,
            ).data
        )
        rospy.sleep(0.5)
        manual_after = link_sample(get_link)
        manual_distance = planar_distance(manual_before, manual_after)
        if manual_distance < 0.008:
            raise RuntimeError("manual Joy path did not measurably move Gazebo")
        assert_standing(manual_after, "manual movement")
        results["manual_motion"] = {
            "before": manual_before,
            "after": manual_after,
            "planar_displacement_m": manual_distance,
            "status": manual_status,
        }

        set_motion(False)
        final_sample, final_speed = wait_for_planar_stop(get_link)
        assert_standing(final_sample, "final stop")
        results["final_stop"] = {"sample": final_sample, "planar_speed_mps": final_speed}

        process_text = subprocess.check_output(["ps", "-eo", "args="], text=True)
        for forbidden in ("example_lite3_real", "message_transformer"):
            if forbidden in process_text:
                raise RuntimeError("forbidden hardware process detected: %s" % forbidden)
        udp_sockets = sim_process_udp_sockets()
        prohibited_udp = [
            item for item in udp_sockets
            if item["remote_port"] == 43893
            or item["local_port"] in (43892, 43897)
            or item["remote_ip"] in ("192.168.2.1", "192.168.1.120")
        ]
        if prohibited_udp:
            raise RuntimeError("motion-host UDP path detected: %s" % prohibited_udp)
        results["hardware_boundary"] = {
            "real_executable": False,
            "motion_host_bridge": False,
            "motion_host_udp_sockets": 0,
            "benign_sim_process_udp_sockets": udp_sockets,
        }
    finally:
        # The bridge's shutdown hook publishes the final zero. Avoid making a
        # synchronous service call while roslaunch may already be tearing down
        # the master; rospy service calls have no connection timeout.
        try:
            os.killpg(launch.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            try:
                launch.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5.0)
        log_handle.flush()
        log_handle.close()

    with open(log_path, "r", encoding="utf-8", errors="replace") as stream:
        log_text = stream.read()
    failure_markers = (
        "failed to load controller",
        "Controllers start failed",
        "controller setup failed",
        "controller spawner error",
        "receive nan value",
        "exist nan value",
        "process has died",
        "Simulation controller supervisor failed",
    )
    found = [marker for marker in failure_markers if marker.lower() in log_text.lower()]
    if found:
        raise RuntimeError("controller/process failure markers in %s: %s" % (log_path, found))
    results["controller_failure_markers"] = []
    print(json.dumps(results, indent=2, sort_keys=True))
    print("GAZEBO_E2E_PASS log=%s" % log_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("GAZEBO_E2E_FAIL: %s" % exc, file=sys.stderr)
        raise
