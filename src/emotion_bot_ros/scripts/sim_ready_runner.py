#!/usr/bin/env python3
"""Start the Lite3 simulation controller only after Gazebo is genuinely ready."""

import os
import signal
import subprocess
import sys

import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetModelState


CONTROLLERS = {
    "joint_states_controller",
    "FL_HipX", "FL_HipY", "FL_Knee",
    "FR_HipX", "FR_HipY", "FR_Knee",
    "HL_HipX", "HL_HipY", "HL_Knee",
    "HR_HipX", "HR_HipY", "HR_Knee",
}


class SimRunner:
    def __init__(self):
        self.child = None
        rospy.on_shutdown(self.stop)

    def wait_until_ready(self):
        timeout = float(rospy.get_param("~startup_timeout", 120.0))
        deadline = rospy.Time.now().to_sec() + timeout
        rospy.wait_for_service("/gazebo/get_model_state", timeout=timeout)
        rospy.wait_for_service("/lite3_gazebo/controller_manager/list_controllers", timeout=timeout)
        get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        list_controllers = rospy.ServiceProxy(
            "/lite3_gazebo/controller_manager/list_controllers", ListControllers
        )
        wall_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            model_ready = get_model("lite3_gazebo", "world").success
            running = {item.name for item in list_controllers().controller if item.state == "running"}
            if model_ready and CONTROLLERS.issubset(running):
                return
            if rospy.Time.now().to_sec() > deadline:
                raise RuntimeError("timed out waiting for model and controllers")
            wall_rate.sleep()

    def run(self):
        self.wait_until_ready()
        command = ["rosrun", "examples", "example_lite3_sim", "/joy:=/emotion_bot/joy_out"]
        rospy.loginfo("Starting Lite3 simulation controller through the arbitrated Joy path")
        self.child = subprocess.Popen(command)
        while not rospy.is_shutdown() and self.child.poll() is None:
            rospy.sleep(0.2)
        if not rospy.is_shutdown() and self.child.returncode:
            raise RuntimeError("Lite3 simulation controller exited with %d" % self.child.returncode)

    def stop(self):
        if self.child is None or self.child.poll() is not None:
            return
        self.child.send_signal(signal.SIGINT)
        try:
            self.child.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            self.child.terminate()
            try:
                self.child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.child.kill()


def main():
    rospy.init_node("lite3_sim_runner")
    runner = SimRunner()
    try:
        runner.run()
    except rospy.ROSInterruptException:
        # Normal roslaunch shutdown interrupts rospy.sleep while the child is
        # being stopped by the registered shutdown hook.
        pass
    except Exception as exc:
        rospy.logfatal("Simulation controller supervisor failed: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
