#!/usr/bin/env python3
import json
import time
import unittest

import rosnode
import rospy
import rostest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from emotion_bot_ros.contract import loads_state


class CoreIntegrationTest(unittest.TestCase):
    def wait_for(self, topic, msg_type, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                message = rospy.wait_for_message(topic, msg_type, timeout=0.5)
            except rospy.ROSException:
                continue
            if predicate(message):
                return message
        self.fail("timed out waiting for %s" % topic)

    def test_complete_core_safety_flow(self):
        state_message = rospy.wait_for_message("/emotion_bot/state", String, timeout=8.0)
        initial = loads_state(state_message.data)
        self.assertEqual(initial["emotion"], "neutral")

        input_pub = rospy.Publisher("/emotion_bot/input", String, queue_size=10)
        manual_pub = rospy.Publisher("/emotion_bot/manual_joy", Joy, queue_size=10)
        direct_pub = rospy.Publisher("/emotion_bot/expression_cmd", Twist, queue_size=10)
        responses = []
        response_sub = rospy.Subscriber(
            "/emotion_bot/response", String, lambda message: responses.append(message.data), queue_size=10
        )
        rospy.sleep(0.4)
        input_pub.publish(String(data="I am thrilled and joyful!"))
        changed = self.wait_for(
            "/emotion_bot/state", String,
            lambda msg: loads_state(msg.data)["sequence"] > initial["sequence"],
        )
        changed_state = loads_state(changed.data)
        self.assertEqual(changed_state["emotion"], "joy")
        self.assertTrue(-1.0 <= changed_state["valence"] <= 1.0)
        self.assertTrue(0.0 <= changed_state["arousal"] <= 1.0)
        deadline = time.monotonic() + 3.0
        while not responses and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(responses)

        # A new subscriber receives the latched contract immediately.
        latched = rospy.wait_for_message("/emotion_bot/state", String, timeout=1.0)
        self.assertGreaterEqual(loads_state(latched.data)["sequence"], changed_state["sequence"])

        disabled = self.wait_for(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0,
        )
        self.assertEqual(disabled.linear.z, 0.0)

        rospy.wait_for_service("/emotion_bot/set_motion_enabled", timeout=5.0)
        set_motion = rospy.ServiceProxy("/emotion_bot/set_motion_enabled", SetBool)
        self.assertTrue(set_motion(True).success)

        # Inject a malformed/oversize internal command; every observed safe
        # command remains finite and bounded.
        oversize = Twist()
        oversize.linear.x = 10.0
        oversize.linear.y = -10.0
        oversize.angular.z = 10.0
        for _ in range(5):
            direct_pub.publish(oversize)
            rospy.sleep(0.05)
        bounded = self.wait_for(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: abs(msg.linear.x) > 0.0,
        )
        self.assertLessEqual(abs(bounded.linear.x), 0.10)
        self.assertLessEqual(abs(bounded.linear.y), 0.05)
        self.assertLessEqual(abs(bounded.angular.z), 0.10)
        self.assertEqual(bounded.linear.z, 0.0)
        self.assertEqual(bounded.angular.x, 0.0)
        self.assertEqual(bounded.angular.y, 0.0)

        manual = Joy()
        manual.axes = [0.0] * 8
        manual.buttons = [0] * 11
        manual.axes[4] = -0.4
        for _ in range(4):
            manual_pub.publish(manual)
            rospy.sleep(0.05)
        status = self.wait_for(
            "/emotion_bot/status", String,
            lambda msg: json.loads(msg.data)["selected_source"] == "manual",
        )
        self.assertEqual(json.loads(status.data)["last_safety_action"], "manual_priority")
        joy = self.wait_for(
            "/emotion_bot/joy_out", Joy,
            lambda msg: len(msg.axes) >= 5 and msg.axes[4] < 0.0,
        )
        self.assertGreaterEqual(joy.axes[4], -1.0)

        # Killing the mapper simulates loss of the selected emotion command.
        rosnode.kill_nodes(["/emotion_bot/expression_mapper"])
        zero = self.wait_for(
            "/emotion_bot/safe_cmd", Twist,
            lambda msg: msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0,
            timeout=4.0,
        )
        self.assertEqual(zero.linear.x, 0.0)

        # Adapter loss cannot re-enable motion or leave a retained command.
        rosnode.kill_nodes(["/emotion_bot/adapter"])
        rospy.sleep(0.8)
        safe_after_adapter_loss = rospy.wait_for_message("/emotion_bot/safe_cmd", Twist, timeout=2.0)
        self.assertEqual(safe_after_adapter_loss.linear.x, 0.0)
        self.assertTrue(set_motion(False).success)


if __name__ == "__main__":
    rospy.init_node("emotion_bot_core_integration_test")
    rostest.rosrun("emotion_bot_ros", "emotion_bot_core_integration", CoreIntegrationTest)
