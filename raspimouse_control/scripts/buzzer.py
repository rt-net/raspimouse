#!/usr/bin/env python
# encoding: utf8

# This script is adapted from
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/scripts/buzzer.py
# which is released under the BSD 3-Clause "New" or "Revised" License.
# Copyright (c) 2017, Ryuichi Ueda
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE

import rospy
import actionlib
from std_msgs.msg import UInt16
from raspimouse_msgs.msg import MusicAction, MusicResult, MusicFeedback

DEVICE_FILE = "/dev/rtbuzzer0"


def write_freq(hz=0):
    bfile = DEVICE_FILE
    try:
        with open(bfile, "w") as f:
            f.write(str(hz) + "\n")
    except IOError:
        rospy.logerr("can't write to " + bfile)


def exec_music(goal):
    r = MusicResult()
    fb = MusicFeedback()

    for i, f in enumerate(goal.freqs):
        fb.remaining_steps = len(goal.freqs) - i
        music.publish_feedback(fb)

        if music.is_preempt_requested():
            write_freq(0)
            r.finished = False
            music.set_preempted(r)
            return

        write_freq(f)
        rospy.sleep(1.0 if i >= len(goal.durations) else goal.durations[i])

    r.finished = True
    music.set_succeeded(r)


def recv_buzzer(data):
    write_freq(data.data)


if __name__ == "__main__":
    rospy.init_node("buzzer")

    DEVICE_FILE = rospy.get_param("~device_file", "/dev/rtbuzzer0")

    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
    music = actionlib.SimpleActionServer(
        "music", MusicAction, exec_music, False
        )
    music.start()
    rospy.on_shutdown(write_freq)
    rospy.spin()
