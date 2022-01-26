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
#
# https://github.com/groove-x/mqtt_bridge/blob/db4ee39da436e60fb3d4557cc1122ec822b65668/scripts/mqtt_bridge_node_test.py
# which is released under the MIT License.
# Copyright (c) 2016 GROOVE X, Inc.
# https://github.com/groove-x/mqtt_bridge/blob/db4ee39da436e60fb3d4557cc1122ec822b65668/LICENSE.txt

import unittest
import rostest
import rosnode
import rosgraph
import rospy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

DEVICE_FILE_MOTOR_SW = '/dev/rtmotoren0'
DEVICE_FILE_RIGHT_MOTOR = '/dev/rtmotor_raw_r0'
DEVICE_FILE_LEFT_MOTOR = '/dev/rtmotor_raw_l0'


class MotorTest(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        on = rospy.ServiceProxy('/motor_on', Trigger)
        _ = on()

        self._odom = Odometry()

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

    def _file_check(self, file, value, message):
        with open(file, "r") as f:
            s = f.readline()
        self.assertEqual(s, str(value)+"\n",
                         "{}, value: {}".format(message, s))

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/motors', nodes, "node does not exist")

    def test_put_cmd_vel(self):
        pub = self._get_publisher(
            '/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        m = Twist()
        # rtmotor_raw_value = (400/(48*math.pi/1000))*m.linear.x
        m.linear.x = 0.1508
        # diff_rtmotor_raw_value = (400/(96*math.pi/92.5))*m.angular.z
        m.angular.z = 1.63
        for i in range(10):
            pub.publish(m)
            time.sleep(0.1)

        self._file_check(DEVICE_FILE_LEFT_MOTOR, 200,
                         "wrong left value from cmd_vel")
        self._file_check(DEVICE_FILE_RIGHT_MOTOR, 600,
                         "wrong right value from cmd_vel")

        time.sleep(1.1)
        self._file_check(DEVICE_FILE_LEFT_MOTOR, 0, "don't stop after 1[s]")
        self._file_check(DEVICE_FILE_RIGHT_MOTOR, 0, "don't stop after 1[s]")

    def test_on_off(self):
        off = rospy.ServiceProxy('/motor_off', Trigger)
        ret = off()
        self.assertEqual(ret.success, True, "motor off does not succeeded")
        self.assertEqual(ret.message, "OFF", "motor off wrong message")
        self._file_check(DEVICE_FILE_MOTOR_SW, 0,
                         "wrong value in rtmotoren0 at motor off")

        on = rospy.ServiceProxy('/motor_on', Trigger)
        ret = on()
        self.assertEqual(ret.success, True, "motor on does not succeeded")
        self.assertEqual(ret.message, "ON", "motor on wrong message")
        self._file_check(DEVICE_FILE_MOTOR_SW, 1,
                         "wrong value in rtmotoren0 at motor off")


if __name__ == '__main__':
    rospy.init_node('test_motors')
    DEVICE_FILE_MOTOR_SW = rospy.get_param(
        '~device_file_motor_sw', '/tmp/rtmotoren0')
    DEVICE_FILE_LEFT_MOTOR = rospy.get_param(
        '~device_file_left_motor', '/tmp/rtmotor_raw_l0')
    DEVICE_FILE_RIGHT_MOTOR = rospy.get_param(
        '~device_file_right_motor', '/tmp/rtmotor_raw_r0')
    rostest.rosrun('raspimouse_control', 'test_motors', MotorTest)
