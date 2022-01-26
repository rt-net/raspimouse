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
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/test/travis_test_motors.py
# which is released under the BSD 3-Clause "New" or "Revised" License.
# Copyright (c) 2017, Ryuichi Ueda
# https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE

import unittest
import rostest
import rosnode
import rospy
import time
from raspimouse_msgs.msg import LightSensorValues

DEVICE_FILE = '/dev/rtlightsensor0'


class LightsensorTest(unittest.TestCase):
    def _check_values(self, message, lf, ls, rs, rf):
        self.assertEqual(message.left_forward, lf, "different value: left_forward")
        self.assertEqual(message.left_side, ls, "different value: left_side")
        self.assertEqual(message.right_side, rs, "different value: right_side")
        self.assertEqual(message.right_forward, rf,
                         "different value: right_forward")
        self.assertEqual(message.sum_all, lf+ls+rs+rf, "different value: sum_all")
        self.assertEqual(message.sum_forward, lf+rf, "different value: sum_forward")

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/lightsensors', nodes, "node does not exist")

    def test_get_value(self):
        rospy.set_param('/lightsensors/frequency', 10)  # センサの値取得の周期を10Hzに
        time.sleep(2)  # パラメータの反映を待つ
        with open(DEVICE_FILE, "w") as f:  # ダミーの値をダミーのファイルに
            f.write("-1 0 123 4321\n")
        # 値が取得できているかを確認
        message = rospy.wait_for_message("/lightsensors", LightSensorValues, timeout=3)
        self._check_values(message, 4321, 123, 0, -1)


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_lightsensors')
    DEVICE_FILE = rospy.get_param('~device_file', '/tmp/rtlightsensor0')
    rostest.rosrun('raspimouse_control', 'test_lightsensors', LightsensorTest)
