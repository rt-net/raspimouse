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

import unittest
import rostest
import rosnode
import rospy
import time
from raspimouse_msgs.msg import ButtonValues

DEVICE_FILE_0 = '/dev/rtswitch0'  # front
DEVICE_FILE_1 = '/dev/rtswitch1'  # middle
DEVICE_FILE_2 = '/dev/rtswitch2'  # rear


class ButtonTest(unittest.TestCase):
    def _check_values(self, message, front, middle, rear):
        self.assertEqual(message.front, front,
                         "different value: front ({} and {})".format(
                             message.front, front))
        self.assertEqual(message.mid, middle,
                         "different value: middle ({} and {})".format(
                             message.mid, middle))
        self.assertEqual(message.rear, rear,
                         "different value: rear ({} and {})".format(
                             message.rear, rear))

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/buttons', nodes, "node does not exist")

    def test_get_value(self):
        with open(DEVICE_FILE_0, "w") as f0, open(DEVICE_FILE_1, "w") as f1, \
             open(DEVICE_FILE_2, "w") as f2:
            f0.write("1\n")
            f1.write("0\n")
            f2.write("0\n")
        message = rospy.wait_for_message("/buttons", ButtonValues, timeout=3)
        self._check_values(message, False, True, True)

        with open(DEVICE_FILE_0, "w") as f0, open(DEVICE_FILE_1, "w") as f1, \
             open(DEVICE_FILE_2, "w") as f2:
            f0.write("0\n")
            f1.write("1\n")
            f2.write("1\n")
        message = rospy.wait_for_message("/buttons", ButtonValues, timeout=3)
        self._check_values(message, True, False, False)


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_buttons')
    DEVICE_FILE_0 = rospy.get_param('~device_file_0', '/tmp/rtswtich0')
    DEVICE_FILE_1 = rospy.get_param('~device_file_1', '/tmp/rtswtich1')
    DEVICE_FILE_2 = rospy.get_param('~device_file_2', '/tmp/rtswtich2')
    rostest.rosrun('raspimouse_control', 'test_buttons', ButtonTest)
