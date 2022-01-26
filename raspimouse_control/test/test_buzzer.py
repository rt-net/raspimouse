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

DEVICE_FILE = '/dev/rtbuzzer0'


class BuzzerTest(unittest.TestCase):
    def _get_subscribers(self, topic_path):
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
        num_subs = len(self._get_subscribers(topic_path))
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
        pub = self._get_publisher('/buzzer', UInt16, queue_size=10)
        for i in range(10):
            pub.publish(1234)
            time.sleep(0.1)

        with open(DEVICE_FILE, "r") as f:
            data = f.readline()
        self.assertEqual(
            data, "1234\n", "value does not written to " + DEVICE_FILE)


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_buzzer')
    DEVICE_FILE = rospy.get_param('~device_file', '/tmp/rtbuzzer0')
    rostest.rosrun('raspimouse_control', 'test_buzzer', BuzzerTest)
