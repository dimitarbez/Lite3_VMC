#!/usr/bin/env python3
"""Forward validated emotion state to an SSH-forwarded loopback endpoint."""

import socket
import threading
import time

import rospy
from std_msgs.msg import String

from emotion_bot_ros.contract import ContractError, loads_state
from emotion_bot_ros.hardware_transport import build_envelope, encode_envelope, new_session_id


class HardwareUplink:
    def __init__(self):
        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 8767))
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        if self.host not in ("127.0.0.1", "::1", "localhost"):
            raise RuntimeError("hardware uplink may connect only to loopback")
        self.session_id = new_session_id()
        self.sequence = 0
        self.state = None
        self.lock = threading.Lock()
        self.socket = None
        rospy.Subscriber(
            rospy.get_param("/emotion_bot/topics/state", "/emotion_bot/state"),
            String,
            self.on_state,
            queue_size=10,
        )
        rospy.on_shutdown(self.close)

    def on_state(self, message):
        try:
            state = loads_state(message.data)
        except ContractError as exc:
            rospy.logwarn("Hardware uplink rejected state: %s", exc)
            return
        with self.lock:
            self.state = state

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None

    def connect(self):
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=0.5)
        sock.settimeout(0.5)
        self.socket = sock

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                state = self.state
            if state is not None:
                try:
                    if self.socket is None:
                        self.connect()
                    self.sequence += 1
                    envelope = build_envelope(self.session_id, self.sequence, state)
                    self.socket.sendall(encode_envelope(envelope))
                except (OSError, ValueError) as exc:
                    self.close()
                    rospy.logwarn_throttle(2.0, "Hardware uplink unavailable: %s", exc)
            rate.sleep()


def main():
    rospy.init_node("hardware_uplink")
    HardwareUplink().run()


if __name__ == "__main__":
    main()
