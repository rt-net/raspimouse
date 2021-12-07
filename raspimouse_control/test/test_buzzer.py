#!/usr/bin/env python
# encoding: utf8

# SPDX-License-Identifier: Apache-2.0

# Copyright 2021 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This script is adapted from
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/test/travis_test_buzzer.py
# which is released under the BSD 3-Clause "New" or "Revised" License.
# Copyright (c) 2017, Ryuichi Ueda
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE

import rospy
import unittest
import rostest
import actionlib
import rosgraph
import rosnode
import time
from std_msgs.msg import UInt16
from raspimouse_msgs.msg import MusicAction


class BuzzerTest(unittest.TestCase):
    def setUp(self):
        self.client = actionlib.SimpleActionClient("music", MusicAction)
        self.device_values = []

    def __get_subscribers(self, topic_path):
        ros_master = rosgraph.Master('/rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend(sub[1])
        return subs

    def _get_publisher(self, topic_path, msg_type, **kwargs):
        # wait until the number of connections would be same as ros master
        pub = rospy.Publisher(topic_path, msg_type, **kwargs)
        num_subs = len(self.__get_subscribers(topic_path))
        for i in range(20):
            num_cons = pub.get_num_connections()
            if num_cons == num_subs:
                return pub
            time.sleep(0.1)
        self.fail("failed to get publisher")

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/buzzer', nodes, "node does not exist")

    def test_put_value(self):
        pub = self._get_publisher('/buzzer', UInt16)
        for i in range(10):
            pub.publish(1234)
            time.sleep(0.1)

        with open("/dev/rtbuzzer0", "r") as f:
            data = f.readline()
            self.assertEqual(
                data, "1234\n", "value does not written to rtbuzzer0")

    def feedback_cb(self, feedback):
        with open("/dev/rtbuzzer0", "r") as f:
            data = f.readline()
            self.device_values.append(int(data.rstrip()))


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_buzzer')
    rostest.rosrun('raspimouse_control', 'test_buzzer', BuzzerTest)