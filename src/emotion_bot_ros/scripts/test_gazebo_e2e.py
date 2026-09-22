#!/usr/bin/env python3
"""Launch and verify the complete headless Gazebo stack with cleanup and logs."""

import argparse
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
    "/emotion_bot/chat_adapter",
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
MIN_STANDING_HEIGHT_M = 0.16
MAX_STANDING_HEIGHT_M = 0.40
EMOTION_RESET_SECONDS = 0.60
EMOTION_SEQUENCE = (
    "neutral", "joy", "sadness", "anger", "fear", "surprise", "disgust", "curiosity", "affection",
)
TRANSITION_SECONDS = {
    "neutral": 0.25,
    "joy": 1.13,
    "sadness": 0.66,
    "anger": 1.47,
    "fear": 0.48,
    "surprise": 0.75,
    "disgust": 0.60,
    "curiosity": 0.50,
    "affection": 0.90,
}
IDLE_SECONDS = {
    "neutral": 1.375,
    "joy": 1.40,
    "sadness": 1.875,
    "anger": 2.4375,
    "fear": 0.45,
    "surprise": 1.60,
    "disgust": 1.90,
    "curiosity": 1.20,
    "affection": 2.75,
}
JOINT_LIMITS = {
    "HipX": (-0.523, 0.523),
    "HipY": (-2.67, 0.314),
    "Knee": (0.524, 2.792),
}


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


def unused_tcp_port():
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


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
    orientation = response.link_state.pose.orientation
    twist = response.link_state.twist
    roll = math.atan2(
        2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
        1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y),
    )
    pitch = math.asin(max(-1.0, min(
        1.0,
        2.0 * (orientation.w * orientation.y - orientation.z * orientation.x),
    )))
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "vx": twist.linear.x,
        "vy": twist.linear.y,
        "vz": twist.linear.z,
        "yaw_rate": twist.angular.z,
        "roll": roll,
        "pitch": pitch,
    }


def planar_distance(first, second):
    return math.hypot(second["x"] - first["x"], second["y"] - first["y"])


def posture_distance(first, second):
    return max(
        abs(second["z"] - first["z"]),
        abs(second["roll"] - first["roll"]),
        abs(second["pitch"] - first["pitch"]),
    )


def joint_distance(first, second):
    """Largest named joint displacement; works across message ordering."""
    before = dict(zip(first.name, first.position))
    after = dict(zip(second.name, second.position))
    shared = set(before).intersection(after)
    return max((abs(after[name] - before[name]) for name in shared), default=0.0)


def assert_joint_margin(message, margin=0.05):
    for name, position in zip(message.name, message.position):
        suffix = name.split("_")[-1]
        if suffix not in JOINT_LIMITS:
            continue
        lower, upper = JOINT_LIMITS[suffix]
        if not lower + margin <= position <= upper - margin:
            raise RuntimeError(
                "%s=%.4f violated %.2f-rad joint-limit margin" % (name, position, margin)
            )


def front_hips_mirrored(message, tolerance=0.12):
    # Independent Gazebo foot contacts can briefly separate measured hips.
    # Keep this measured-error guard alongside individual HipX and URDF checks.
    positions = dict(zip(message.name, message.position))
    if "FL_HipX" not in positions or "FR_HipX" not in positions:
        raise RuntimeError("front hip joints missing during stomp")
    error = abs(positions["FL_HipX"] + positions["FR_HipX"])
    if error > tolerance:
        raise RuntimeError("front hip mirror error %.4f exceeded %.4f" % (error, tolerance))
    return error


def assert_hips_not_splayed(message, maximum=0.31):
    # Allow 0.01 rad of Gazebo contact transient above the nominal 0.30 rad
    # anti-splay target. The separate URDF-margin check remains stricter than
    # the actual joint limit throughout the animation sweep.
    positions = dict(zip(message.name, message.position))
    for name in ("FL_HipX", "FR_HipX", "HL_HipX", "HR_HipX"):
        if name not in positions:
            raise RuntimeError("HipX joints missing")
        if abs(positions[name]) > maximum:
            raise RuntimeError(
                "%s=%.4f exceeded %.2f-rad anti-splay limit"
                % (name, positions[name], maximum)
            )


def twist_is_neutral(message, tolerance=1e-4):
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(abs(value) <= tolerance for value in values)


def assert_standing(sample, phase):
    if not all(math.isfinite(value) for value in sample.values()):
        raise RuntimeError("non-finite torso state during %s" % phase)
    if not MIN_STANDING_HEIGHT_M <= sample["z"] <= MAX_STANDING_HEIGHT_M:
        raise RuntimeError(
            "Lite3 torso height %.4f m outside standing range [%.2f, %.2f] during %s"
            % (sample["z"], MIN_STANDING_HEIGHT_M, MAX_STANDING_HEIGHT_M, phase)
        )
    if max(abs(sample["roll"]), abs(sample["pitch"])) > 0.60:
        raise RuntimeError(
            "Lite3 tilt (roll %.4f, pitch %.4f) exceeded 0.60 rad during %s"
            % (sample["roll"], sample["pitch"], phase)
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
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--animation",
        action="store_true",
        help="hold every centered profile through two complete idle cycles",
    )
    args, _unknown = parser.parse_known_args()
    # Never attach this destructive lifecycle test to a developer's running
    # graph. Isolated ROS/Gazebo masters also allow planted and animation runs
    # to execute without touching an interactive GUI session.
    ros_port = unused_tcp_port()
    gazebo_port = unused_tcp_port()
    while gazebo_port == ros_port:
        gazebo_port = unused_tcp_port()
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % ros_port
    os.environ["GAZEBO_MASTER_URI"] = "http://127.0.0.1:%d" % gazebo_port
    log_handle = tempfile.NamedTemporaryFile(
        prefix="emotion_bot_gazebo_e2e_", suffix=".log", delete=False, mode="w+"
    )
    log_path = log_handle.name
    command = [
        "roslaunch", "emotion_bot_ros", "integrated_sim.launch",
        "gui:=false", "headless:=true", "motion_enabled:=false",
        "allow_locomotion:=%s" % ("true" if args.animation else "false"),
    ]
    launch = subprocess.Popen(
        command,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    results = {
        "log": log_path,
        "mode": "animation" if args.animation else "planted",
        "ros_master_port": ros_port,
        "gazebo_master_port": gazebo_port,
    }
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
        prepared_status = json.loads(
            wait_message(
                "/emotion_bot/status", String,
                lambda msg: json.loads(msg.data).get("stance_prepared", False),
                timeout=90.0,
            ).data
        )
        if prepared_status.get("motion_enabled") or not prepared_status.get("health_ok"):
            raise RuntimeError("unsafe prepared-stance status: %s" % prepared_status)
        results["prepared_stance"] = prepared_status

        rospy.wait_for_service("/gazebo/get_link_state", timeout=30.0)
        get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        rospy.wait_for_service("/emotion_bot/set_motion_enabled", timeout=20.0)
        set_motion = rospy.ServiceProxy("/emotion_bot/set_motion_enabled", SetBool)
        input_pub = rospy.Publisher("/emotion_bot/chat/input", String, queue_size=10)
        direct_pub = rospy.Publisher("/emotion_bot/expression_cmd", Twist, queue_size=10)
        manual_pub = rospy.Publisher("/emotion_bot/manual_joy", Joy, queue_size=10)
        responses = []
        states = []
        safe_commands = []
        joint_samples = []
        stomp_events = []
        statuses = []
        def remember_joint(message):
            joint_samples[:] = [message]
        def remember_joy(message):
            if len(message.buttons) > 6 and message.buttons[6]:
                cancelled_at = rospy.Time.now()
                for event in stomp_events:
                    if not event["recovered"] and event["cancelled_at"] is None:
                        event["cancelled_at"] = cancelled_at
            if len(message.buttons) > 5 and message.buttons[5] and joint_samples:
                stomp_events.append({
                    "started": rospy.Time.now(),
                    "before": joint_samples[-1],
                    "recovered": False,
                    "cancelled_at": None,
                    "mirror_peak": 0.0,
                    "mirror_samples": 0,
                })
        response_sub = rospy.Subscriber(
            "/emotion_bot/chat/response", String, lambda message: responses.append(message.data), queue_size=10
        )
        state_sub = rospy.Subscriber(
            "/emotion_bot/state", String,
            lambda message: states.append(json.loads(message.data)), queue_size=20,
        )
        safe_sub = rospy.Subscriber(
            "/emotion_bot/safe_cmd", Twist, lambda message: safe_commands.append(message), queue_size=200
        )
        status_sub = rospy.Subscriber(
            "/emotion_bot/status", String,
            lambda message: statuses.append(json.loads(message.data)), queue_size=20,
        )
        joint_sub = rospy.Subscriber(
            "/lite3_gazebo/joint_states", JointState, remember_joint, queue_size=1
        )
        joy_sub = rospy.Subscriber(
            "/emotion_bot/joy_out", Joy, remember_joy, queue_size=20
        )

        def check_animation_joints(phase):
            if not joint_samples:
                return
            current = joint_samples[-1]
            assert_hips_not_splayed(current)
            if not args.animation:
                return
            try:
                assert_joint_margin(current)
            except RuntimeError as exc:
                pose = link_sample(get_link)
                raise RuntimeError(
                    "%s during %s (torso roll %.4f, pitch %.4f)"
                    % (exc, phase, pose["roll"], pose["pitch"])
                )
            now = rospy.Time.now()
            for event in stomp_events:
                # Retargeting cancels an in-flight stomp. The controller then
                # blends toward the new posture, so the active-stomp mirror
                # and same-pose recovery checks no longer apply to that event.
                if event["cancelled_at"] is not None and now >= event["cancelled_at"]:
                    continue
                elapsed = (now - event["started"]).to_sec()
                # The controller intentionally blends the previous body pose
                # out over its first 0.08 s. Allow additional actuator lag
                # before asserting symmetric measured hips during the stomp.
                if 0.15 <= elapsed <= 0.75:
                    try:
                        error = front_hips_mirrored(current)
                    except RuntimeError as exc:
                        raise RuntimeError(
                            "%s at %.3f s after stomp pulse during %s"
                            % (exc, elapsed, phase)
                        )
                    event["mirror_peak"] = max(event["mirror_peak"], error)
                    event["mirror_samples"] += 1
                elif elapsed >= 1.90 and not event["recovered"]:
                    if event["mirror_samples"] < 2:
                        raise RuntimeError(
                            "stomp had only %d measured mirror samples"
                            % event["mirror_samples"]
                        )
                    recovery_error = joint_distance(event["before"], current)
                    if recovery_error > 0.30:
                        raise RuntimeError(
                            "stomp recovery joint error %.4f exceeded 0.30 rad" % recovery_error
                        )
                    event["recovered"] = True

        def assert_runtime_healthy(phase):
            if not statuses:
                return
            status = statuses[-1]
            if not status.get("health_ok", False):
                raise RuntimeError(
                    "simulation health fault %s during %s"
                    % (status.get("health_fault", "unknown"), phase)
                )
            if not status.get("motion_enabled", False):
                raise RuntimeError("motion permission was revoked during %s" % phase)
        rospy.sleep(0.5)

        initial_state = json.loads(wait_message("/emotion_bot/state", String).data)
        initial_emotion = "neutral" if args.animation else "joy"
        input_pub.publish(String(data=json.dumps({
            "turn_id": "gazebo-turn", "text": "event:%s" % initial_emotion,
        })))
        changed = wait_wall(
            lambda: next(
                (
                    state for state in states
                    if state.get("turn_id") == "gazebo-turn" and state.get("source") == "user"
                ),
                None,
            ),
            "turn-scoped user emotion state",
            timeout=10.0,
        )
        if changed["emotion"] != initial_emotion:
            raise RuntimeError(
                "expected %s, got %s" % (initial_emotion, changed["emotion"])
            )
        wait_wall(lambda: responses[-1] if responses else None, "emotion response", timeout=5.0)
        response = json.loads(responses[-1])["text"]
        results["emotion"] = changed
        results["response"] = response

        before = link_sample(get_link)
        assert_standing(before, "pre-expression")
        if not set_motion(True).success:
            raise RuntimeError("motion enable service failed")
        safe = wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: max(abs(msg.linear.z), abs(msg.angular.x), abs(msg.angular.y))
            > (0.0005 if args.animation else 0.005),
            timeout=30.0,
        )
        if (
            (not args.animation and (
                abs(safe.linear.x) > 1e-9
                or abs(safe.linear.y) > 1e-9
                or abs(safe.angular.z) > 1e-9
            ))
            or abs(safe.linear.x) > 0.100001
            or abs(safe.linear.y) > 0.050001
            or abs(safe.angular.z) > 0.100001
            or abs(safe.linear.z) > 0.100001
            or abs(safe.angular.x) > 0.625001
            or abs(safe.angular.y) > 0.625001
        ):
            raise RuntimeError("unsafe expression command observed")

        peak_distance = 0.0
        peak_speed = 0.0
        moved_sample = before
        movement_minimum = 0.001 if args.animation else 0.008
        movement_deadline = time.monotonic() + 45.0
        while time.monotonic() < movement_deadline:
            sample = link_sample(get_link)
            distance = posture_distance(before, sample)
            speed = math.hypot(sample["vx"], sample["vy"])
            if distance > peak_distance:
                peak_distance = distance
                moved_sample = sample
            peak_speed = max(peak_speed, speed)
            if peak_distance >= movement_minimum:
                break
            time.sleep(0.4)
        if peak_distance < movement_minimum:
            raise RuntimeError("Gazebo body posture did not measurably move")
        assert_standing(moved_sample, "emotion movement")
        results["emotion_motion"] = {
            "before": before,
            "after": moved_sample,
            "posture_displacement": peak_distance,
            "planar_displacement_m": planar_distance(before, moved_sample),
            "peak_planar_speed_mps": peak_speed,
        }

        # Let the preliminary posture settle before starting the independent
        # per-profile measurements. Animation mode uses neutral here so every
        # theatrical entrance is exercised exactly once in the sequence below.
        initial_settle_end = rospy.Time.now() + rospy.Duration(1.0)
        initial_settle_deadline = time.monotonic() + 45.0
        while rospy.Time.now() < initial_settle_end and time.monotonic() < initial_settle_deadline:
            sample = link_sample(get_link)
            assert_standing(sample, "opening expression settle")
            assert_runtime_healthy("opening expression settle")
            time.sleep(0.20)
        if rospy.Time.now() < initial_settle_end:
            raise RuntimeError("simulated time stalled during opening expression settle")

        # Exercise every profile through ROS state publication, rather than
        # merely observing mapper topics.  Each test starts from the torso's
        # current physical pose, samples its entrance peak, then samples after
        # the configured transition has completed to prove the idle loop keeps
        # expressing the active emotion.  Neutral is intentionally allowed to
        # settle to exact zero; every other profile must visibly move in Gazebo.
        profile_results = {}
        previous_emotion = None
        for emotion in EMOTION_SEQUENCE:
            print("Reviewing %s (%s)" % (emotion, results["mode"]), flush=True)
            profile_stomp_start = len(stomp_events)
            profile_before = link_sample(get_link)
            joints_before = wait_message("/lite3_gazebo/joint_states", JointState, timeout=5.0)
            marker = "gazebo-profile-%s" % emotion
            safe_commands[:] = []
            input_pub.publish(String(data=json.dumps({"turn_id": marker, "text": "event:%s" % emotion})))
            profile_state = wait_wall(
                lambda: next(
                    (
                        state for state in states
                        if state.get("turn_id") == marker and state.get("source") == "user"
                    ),
                    None,
                ),
                "%s state" % emotion,
                timeout=10.0,
            )
            if profile_state["emotion"] != emotion:
                raise RuntimeError("expected %s state, got %s" % (emotion, profile_state["emotion"]))
            # Attribute bursts only after this category's accepted state.
            # An old-category pulse may already be in transit while the
            # chat adapter processes the new request.
            if emotion != "anger":
                profile_stomp_start = len(stomp_events)
            transition_peak = 0.0
            joint_peak = 0.0
            peak_sample = profile_before
            # End the entrance window at the profile's actual configured
            # boundary in both modes. A fixed four-second delay can land the
            # following idle baseline at an arbitrary point in a short loop
            # (especially fear's short tremble), making the measured
            # excursion depend on phase alignment rather than expressiveness.
            reset_required = previous_emotion not in (None, "neutral") and emotion != "neutral"
            transition_seconds = TRANSITION_SECONDS[emotion]
            if reset_required:
                transition_seconds += EMOTION_RESET_SECONDS
            transition_end = rospy.Time.now() + rospy.Duration(transition_seconds)
            wall_deadline = time.monotonic() + max(30.0, transition_seconds * 10.0)
            planar_peak = 0.0
            while rospy.Time.now() < transition_end and time.monotonic() < wall_deadline:
                sample = link_sample(get_link)
                change = posture_distance(profile_before, sample)
                planar_peak = max(planar_peak, planar_distance(profile_before, sample))
                if change > transition_peak:
                    transition_peak = change
                    peak_sample = sample
                if joint_samples:
                    joint_peak = max(joint_peak, joint_distance(joints_before, joint_samples[-1]))
                check_animation_joints("%s transition" % emotion)
                assert_standing(sample, "%s transition" % emotion)
                assert_runtime_healthy("%s transition" % emotion)
                time.sleep(0.20)
            if rospy.Time.now() < transition_end:
                raise RuntimeError("simulated time stalled during %s transition" % emotion)
            if reset_required:
                longest_neutral_run = 0
                current_neutral_run = 0
                for command in safe_commands:
                    if twist_is_neutral(command):
                        current_neutral_run += 1
                        longest_neutral_run = max(longest_neutral_run, current_neutral_run)
                    else:
                        current_neutral_run = 0
                if longest_neutral_run < 5:
                    raise RuntimeError(
                        "%s did not hold a fresh canonical stance before its entrance"
                        % emotion
                    )
            idle_before = link_sample(get_link)
            idle_joints_before = wait_message("/lite3_gazebo/joint_states", JointState, timeout=5.0)
            idle_seconds = 2.0 * IDLE_SECONDS[emotion] if args.animation else 3.0
            idle_end = rospy.Time.now() + rospy.Duration(idle_seconds)
            wall_deadline = time.monotonic() + max(30.0, idle_seconds * 10.0)
            idle_change = 0.0
            idle_joint_change = 0.0
            idle_after = idle_before
            while rospy.Time.now() < idle_end and time.monotonic() < wall_deadline:
                idle_after = link_sample(get_link)
                planar_peak = max(planar_peak, planar_distance(profile_before, idle_after))
                idle_change = max(idle_change, posture_distance(idle_before, idle_after))
                if joint_samples:
                    idle_joint_change = max(
                        idle_joint_change,
                        joint_distance(idle_joints_before, joint_samples[-1]),
                    )
                check_animation_joints("%s idle" % emotion)
                assert_standing(idle_after, "%s idle" % emotion)
                assert_runtime_healthy("%s idle" % emotion)
                time.sleep(0.20)
            if rospy.Time.now() < idle_end:
                raise RuntimeError("simulated time stalled during %s idle" % emotion)
            # These are physical Gazebo measurements, not command-topic
            # deltas. They intentionally reject animations that only move by
            # a few invisible milliradians.
            entrance_minimum = 0.006 if emotion == "neutral" else 0.018
            idle_minimum = 0.004 if emotion == "neutral" else 0.012
            if (args.animation or emotion != "neutral") and max(transition_peak, joint_peak) < entrance_minimum:
                raise RuntimeError(
                    "%s transition was not visibly measurable (torso %.4f, joint %.4f)"
                    % (emotion, transition_peak, joint_peak)
                )
            if (args.animation or emotion != "neutral") and max(idle_change, idle_joint_change) < idle_minimum:
                raise RuntimeError(
                    "%s idle loop was not visibly measurable (torso %.4f, joint %.4f)"
                    % (emotion, idle_change, idle_joint_change)
                )
            # Every chat expression is planted. Expressive hops may produce a
            # few harmless centimeters of contact drift; auto-recenter begins
            # at 0.09 m, so reject sustained travel rather than theatricality.
            planar_limit = 0.10
            if planar_peak > planar_limit:
                raise RuntimeError(
                    "%s exceeded %.3f m planar envelope (%.4f m)"
                    % (emotion, planar_limit, planar_peak)
                )
            emotion_stomps = stomp_events[profile_stomp_start:]
            if args.animation and emotion == "anger":
                if len(emotion_stomps) < 3:
                    raise RuntimeError(
                        "anger emitted only %d stomp bursts during animation hold"
                        % len(emotion_stomps)
                    )
                starts = [item["started"].to_sec() for item in emotion_stomps]
                spacings = [second - first for first, second in zip(starts, starts[1:])]
                recurring = spacings[1:] if len(spacings) > 1 else []
                if not recurring or not all(2.25 <= spacing <= 2.65 for spacing in recurring):
                    raise RuntimeError("anger recurring stomp spacing outside 2.25-2.65 seconds: %s" % spacings)
                if len([item for item in emotion_stomps if item["recovered"]]) < 2:
                    raise RuntimeError("fewer than two anger stomps completed bounded recovery")
            profile_results[emotion] = {
                "state_sequence": profile_state["sequence"],
                "transition_displacement": transition_peak,
                "transition_joint_displacement": joint_peak,
                "idle_variation": idle_change,
                "idle_joint_variation": idle_joint_change,
                "planar_displacement_m": planar_peak,
                "stomp_bursts": len(emotion_stomps),
                "stomp_recoveries": len(
                    [item for item in emotion_stomps if item["recovered"]]
                ),
                "stomp_front_hip_mirror_peak_rad": max(
                    (item["mirror_peak"] for item in emotion_stomps), default=0.0
                ),
                "peak": peak_sample,
            }
            print("  travel=%.4fm, idle joint motion=%.4frad, stomps=%d" % (
                planar_peak, idle_joint_change, len(emotion_stomps)), flush=True)
            previous_emotion = emotion
        results["all_emotion_profiles"] = profile_results

        # Interrupt two entrance gestures, then repeat anger once inside and
        # once beyond its cooldown.  The mapper should blend the rapid change
        # and replay only the cooled-down trigger; all observed transport stays
        # within the same stance-only envelope.
        rapid_sequences = []
        for index, emotion in enumerate(("joy", "anger", "fear")):
            marker = "gazebo-rapid-%d" % index
            input_pub.publish(String(data=json.dumps({"turn_id": marker, "text": "event:%s" % emotion})))
            rapid = wait_wall(
                lambda marker=marker: next(
                    (state for state in states if state.get("turn_id") == marker and state.get("source") == "user"),
                    None,
                ),
                "rapid %s state" % emotion,
                timeout=10.0,
            )
            rapid_sequences.append(rapid["sequence"])
            if index == 0:
                rospy.sleep(0.12)
            elif index == 1:
                rospy.sleep(0.12)
        rospy.sleep(0.75)
        repeat_marker = "gazebo-repeat-fear"
        input_pub.publish(String(data=json.dumps({"turn_id": repeat_marker, "text": "event:fear"})))
        wait_wall(
            lambda: next(
                (state for state in states if state.get("turn_id") == repeat_marker and state.get("source") == "user"),
                None,
            ),
            "repeated fear state",
            timeout=10.0,
        )
        rospy.sleep(0.35)
        if not safe_commands:
            raise RuntimeError("no safe commands observed during rapid emotion test")
        for command in safe_commands[-30:]:
            if (
                abs(command.linear.x) > 1e-9 or abs(command.linear.y) > 1e-9 or abs(command.angular.z) > 1e-9
                or abs(command.linear.z) > 0.100001 or abs(command.angular.x) > 0.625001
                or abs(command.angular.y) > 0.625001
            ):
                raise RuntimeError("rapid emotion change escaped safe posture limits")
        results["rapid_and_repeated"] = {"state_sequences": rapid_sequences, "safe": True}

        set_motion(False)
        wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: all(value == 0.0 for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            )),
            timeout=10.0,
        )
        stopped, stopped_speed = wait_for_planar_stop(get_link)
        assert_standing(stopped, "disabled stop")
        results["disabled_stop"] = {"sample": stopped, "planar_speed_mps": stopped_speed}

        # Remove the mapper, drive the bridge directly, then stop publishing to
        # prove its independent command watchdog reaches zero.
        rosnode.kill_nodes(["/emotion_bot/expression_mapper"])
        set_motion(True)
        safe_commands[:] = []
        direct = Twist()
        direct.linear.z = 0.015
        direct.angular.x = 0.035
        # Keep the stimulus alive past the configured stand transition;
        # then stop publishing so the independent bridge watchdog can expire it.
        publish_for(direct_pub, direct, 2.2)
        wait_wall(
            lambda: any(message.linear.z > 0.005 for message in safe_commands),
            "bounded direct command before watchdog expiry",
            timeout=10.0,
        )
        stale_zero = wait_message(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: all(value == 0.0 for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            )),
            timeout=15.0,
        )
        status = json.loads(
            wait_message(
                "/emotion_bot/status", String,
                lambda msg: json.loads(msg.data)["stale"],
                timeout=10.0,
            ).data
        )
        results["watchdog"] = {"zero": stale_zero.linear.z == 0.0, "status": status}

        # The planted run also proves that manual locomotion mode is blocked.
        # The animation run keeps locomotion enabled but uses a posture-only
        # manual command here so the final shutdown assertion is comparable.
        manual_before = link_sample(get_link)
        joy = Joy()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 11
        locomotion_button_blocked = None
        if not args.animation:
            joy.buttons[2] = 1
            manual_pub.publish(joy)
            blocked_x = wait_message(
                "/emotion_bot/joy_out", Joy,
                lambda msg: len(msg.buttons) > 2 and msg.buttons[2] == 0,
                timeout=5.0,
            )
            locomotion_button_blocked = blocked_x.buttons[2] == 0
        joy.buttons = [0] * 11
        joy.axes[6] = 0.4
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
        manual_distance = posture_distance(manual_before, manual_after)
        if manual_distance < 0.008:
            raise RuntimeError("manual posture path did not measurably move Gazebo")
        assert_standing(manual_after, "manual movement")
        results["manual_motion"] = {
            "before": manual_before,
            "after": manual_after,
            "posture_displacement": manual_distance,
            "locomotion_button_blocked": locomotion_button_blocked,
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
