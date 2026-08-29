#!/usr/bin/env python3
"""Load and start Lite3 Gazebo controllers once, without teardown races."""

import sys

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    SwitchController,
    SwitchControllerRequest,
)


CONTROLLERS = [
    "joint_states_controller",
    "FL_HipX", "FL_HipY", "FL_Knee",
    "FR_HipX", "FR_HipY", "FR_Knee",
    "HL_HipX", "HL_HipY", "HL_Knee",
    "HR_HipX", "HR_HipY", "HR_Knee",
]


def main():
    rospy.init_node("lite3_controller_loader")
    namespace = "/lite3_gazebo/controller_manager"
    timeout = float(rospy.get_param("~startup_timeout", 90.0))
    service_names = {
        "list": namespace + "/list_controllers",
        "load": namespace + "/load_controller",
        "switch": namespace + "/switch_controller",
    }
    try:
        for service_name in service_names.values():
            rospy.wait_for_service(service_name, timeout=timeout)
        list_controllers = rospy.ServiceProxy(service_names["list"], ListControllers)
        load_controller = rospy.ServiceProxy(service_names["load"], LoadController)
        switch_controller = rospy.ServiceProxy(service_names["switch"], SwitchController)

        states = {item.name: item.state for item in list_controllers().controller}
        for name in CONTROLLERS:
            if name not in states and not load_controller(name).ok:
                raise RuntimeError("failed to load controller %s" % name)

        states = {item.name: item.state for item in list_controllers().controller}
        to_start = [name for name in CONTROLLERS if states.get(name) != "running"]
        if to_start:
            response = switch_controller(
                to_start,
                [],
                SwitchControllerRequest.STRICT,
                False,
                0.0,
            )
            if not response.ok:
                raise RuntimeError("failed to start controllers: %s" % ", ".join(to_start))

        running = {
            item.name for item in list_controllers().controller if item.state == "running"
        }
        missing = set(CONTROLLERS) - running
        if missing:
            raise RuntimeError("controllers not running: %s" % ", ".join(sorted(missing)))
        rospy.loginfo("Loaded and started %d Lite3 simulation controllers", len(CONTROLLERS))
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        rospy.logfatal("Lite3 controller setup failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
