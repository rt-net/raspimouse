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
    def setUp(self):
        self._count = 0
        rospy.Subscriber('/lightsensors', LightSensorValues, self._callback)
        self._values = LightSensorValues()

    def _callback(self, data):
        self._count += 1
        self._values = data

    def _check_values(self, lf, ls, rs, rf):
        vs = self._values
        self.assertEqual(vs.left_forward, lf, "different value: left_forward")
        self.assertEqual(vs.left_side, ls, "different value: left_side")
        self.assertEqual(vs.right_side, rs, "different value: right_side")
        self.assertEqual(vs.right_forward, rf,
                         "different value: right_forward")
        self.assertEqual(vs.sum_all, lf+ls+rs+rf, "different value: sum_all")
        self.assertEqual(vs.sum_forward, lf+rf, "different value: sum_forward")

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/lightsensors', nodes, "node does not exist")

    def test_get_value(self):
        rospy.set_param('/lightsensors/frequency', 10)  # センサの値取得の周期を10Hzに
        time.sleep(2)  # パラメータの反映を待つ
        with open(DEVICE_FILE, "w") as f:  # ダミーの値をダミーのファイルに
            f.write("-1 0 123 4321\n")

        time.sleep(3)
        # コールバック関数が最低1回は呼ばれ、値が取得できているかを確認
        self.assertFalse(self._count == 0, "cannot subscribe the topic")
        self._check_values(4321, 123, 0, -1)

    def test_change_parameter(self):
        rospy.set_param('/lightsensors/frequency', 1)  # センサの値取得の周期を1Hzに
        time.sleep(2)  # 反映を待つ
        c_prev = self._count  # callbackが呼ばれた回数を記録
        time.sleep(3)
        # コールバック関数が3秒間で最低1回、最高でも4回しか呼ばれてないことを確認
        self.assertTrue(self._count < c_prev + 4, "freq does not change")
        self.assertFalse(self._count == c_prev, "subscriber is stopped")


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_lightsensors')
    DEVICE_FILE = rospy.get_param('~device_file', '/tmp/rtlightsensor0')
    rostest.rosrun('raspimouse_control', 'test_lightsensors', LightsensorTest)
