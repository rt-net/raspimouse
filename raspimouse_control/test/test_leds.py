#!/usr/bin/env python
# encoding: utf8

# SPDX-License-Identifier: Apache-2.0

# Copyright 2022 RT Corporation
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

import unittest
import rosgraph
import rostest
import rosnode
import rospy
import time
from raspimouse_msgs.msg import LedValues

DEVICE_FILE_0 = '/dev/rtled0'
DEVICE_FILE_1 = '/dev/rtled1'
DEVICE_FILE_2 = '/dev/rtled2'
DEVICE_FILE_3 = '/dev/rtled3'


class LedTest(unittest.TestCase):
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

    def _check_values(self, message, values):
        self.assertEqual(message.right_side, int(values[0]),
                         "different value: LED0 ({} and {})".format(
                             message.right_side, int(values[0])))
        self.assertEqual(message.right_forward, int(values[1]),
                         "different value: LED1 ({} and {})".format(
                             message.right_forward, int(values[1])))
        self.assertEqual(message.left_forward, int(values[2]),
                         "different value: LED2 ({} and {})".format(
                             message.left_forward, int(values[2])))
        self.assertEqual(message.left_side, int(values[3]),
                         "different value: LED3 ({} and {})".format(
                             message.left_side, int(values[3])))

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/leds', nodes, "node does not exist")

    def test_set_value(self):
        pub = self._get_publisher(
            'leds', LedValues, queue_size=10)

        m = LedValues()
        m.right_side = True
        m.right_forward = False
        m.left_forward = False
        m.left_side = False
        pub.publish(m)
        time.sleep(1)
        with open(DEVICE_FILE_0, "r") as f0, open(DEVICE_FILE_1, "r") as f1, \
             open(DEVICE_FILE_2, "r") as f2, open(DEVICE_FILE_3, "r") as f3:
            (led0, led1, led2, led3) = f0.readline(), f1.readline(), \
                                       f2.readline(), f3.readline()
        self._check_values(m, (led0, led1, led2, led3))

        m = LedValues()
        m.right_side = False
        m.right_forward = True
        m.left_forward = True
        m.left_side = True
        pub.publish(m)
        time.sleep(1)
        with open(DEVICE_FILE_0, "r") as f0, open(DEVICE_FILE_1, "r") as f1, \
             open(DEVICE_FILE_2, "r") as f2, open(DEVICE_FILE_3, "r") as f3:
            (led0, led1, led2, led3) = f0.readline(), f1.readline(), \
                                       f2.readline(), f3.readline()
        self._check_values(m, (led0, led1, led2, led3))


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_leds')
    DEVICE_FILE_0 = rospy.get_param('~device_file_0', '/tmp/rtled0')
    DEVICE_FILE_1 = rospy.get_param('~device_file_1', '/tmp/rtled1')
    DEVICE_FILE_2 = rospy.get_param('~device_file_2', '/tmp/rtled2')
    DEVICE_FILE_3 = rospy.get_param('~device_file_3', '/tmp/rtled3')
    rostest.rosrun('raspimouse_control', 'test_leds', LedTest)
