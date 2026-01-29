#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from cola2_msgs.msg import Setpoints, BodyForceReq


class SetpointsMux(object):
    """
    Multiplex Setpoints:
      - Forward base if base is fresh AND requester priority <= threshold
      - Otherwise forward safety
    """

    def __init__(self):
        # Params
        self.base_topic = rospy.get_param("~base_topic", "~setpoints_mux/base_in")
        self.safety_topic = rospy.get_param("~safety_topic", "~setpoints_mux/safety_in")
        self.requester_topic = rospy.get_param("~requester_topic", "~setpoints_mux/safety_requester")
        self.output_topic = rospy.get_param("~output_topic", "~setpoints_mux/out")

        self.base_timeout = rospy.get_param("~base_timeout", 0.5)  # seconds
        self.priority_threshold = rospy.get_param("~priority_threshold", 40)

        # Publish rate: ensures we switch after timeout even if no new messages arrive exactly then
        self.pub_rate = rospy.get_param("~publish_rate", 20.0)  # Hz

        # State
        self.last_base_msg = None
        self.last_safety_msg = None
        self.last_base_stamp = None
        self.last_safety_stamp = None

        self.last_requester_msg = None
        self.last_requester_stamp = None

        self.last_published_source = None  # "base" or "safety"
        self.last_published_stamp = rospy.Time(0)

        # Pub/Sub
        self.pub = rospy.Publisher(self.output_topic, Setpoints, queue_size=10)

        self.sub_base = rospy.Subscriber(self.base_topic, Setpoints, self._cb_base, queue_size=10)
        self.sub_safety = rospy.Subscriber(self.safety_topic, Setpoints, self._cb_safety, queue_size=10)
        self.sub_requester = rospy.Subscriber(self.requester_topic, BodyForceReq, self._cb_requester, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.pub_rate, 1e-3)), self._on_timer)

        rospy.loginfo(
            "SetpointsMux running:\n"
            "  base_topic=%s\n"
            "  safety_topic=%s\n"
            "  requester_topic=%s\n"
            "  output_topic=%s\n"
            "  base_timeout=%.3fs\n"
            "  priority_threshold=%s\n"
            "  publish_rate=%.1fHz",
            self.base_topic, self.safety_topic, self.requester_topic, self.output_topic,
            self.base_timeout, str(self.priority_threshold), self.pub_rate
        )

    def _cb_base(self, msg):
        self.last_base_msg = msg
        self.last_base_stamp = rospy.Time.now()

    def _cb_safety(self, msg):
        self.last_safety_msg = msg
        self.last_safety_stamp = rospy.Time.now()

    def _cb_requester(self, msg):
        self.last_requester_msg = msg
        self.last_requester_stamp = rospy.Time.now()

    def _base_is_fresh(self, now):
        if self.last_base_stamp is None:
            return False
        return (now - self.last_base_stamp).to_sec() <= float(self.base_timeout)

    def _requester_forces_safety(self):
        if self.last_requester_msg is None:
            return False
        try:
            return self.last_requester_msg.goal.priority >= int(self.priority_threshold)
        except Exception:
            # If something is weird, fail safe by NOT forcing (you can flip this if you prefer)
            return False

    def _choose_source(self, now):
        force_safety = self._requester_forces_safety()
        base_fresh = self._base_is_fresh(now)

        if force_safety:
            return "safety"
        if base_fresh:
            return "base"
        return "safety"

    def _get_msg_for_source(self, source):
        if source == "base":
            return self.last_base_msg
        return self.last_safety_msg

    def _on_timer(self, _evt):
        now = rospy.Time.now()
        source = self._choose_source(now)
        msg = self._get_msg_for_source(source)

        if msg is None:
            # Nothing to publish from selected source
            return

        # Publish continuously at pub_rate (simple & robust)
        self.pub.publish(msg)

        # Optional: log switching events
        if source != self.last_published_source:
            rospy.logwarn("SetpointsMux switched to: %s", source)
            self.last_published_source = source


def main():
    rospy.init_node("setpoints_mux", anonymous=False)
    _ = SetpointsMux()
    rospy.spin()


if __name__ == "__main__":
    main()
