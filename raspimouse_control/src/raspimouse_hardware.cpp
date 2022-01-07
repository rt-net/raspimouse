// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright 2021 RT Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <fstream>
#include "raspimouse_control/raspimouse_hardware.hpp"

RaspberryPiMouseHW::RaspberryPiMouseHW(ros::NodeHandle nh)
{
  std::fill_n(pos, 2, 0.0);
  std::fill_n(vel, 2, 0.0);
  std::fill_n(eff, 2, 0.0);
  nh.getParam("diff_drive_controller/right_wheel", right_wheel_joint_);
  nh.getParam("diff_drive_controller/left_wheel", left_wheel_joint_);
  ROS_INFO("right_wheel_joint: %s, left_wheel_joint: %s", right_wheel_joint_.c_str(), left_wheel_joint_.c_str());

  hardware_interface::JointStateHandle state_right_wheel_handle(right_wheel_joint_, &pos[0], &vel[0], &eff[0]);
  joint_state_interface.registerHandle(state_right_wheel_handle);

  hardware_interface::JointStateHandle state_left_wheel_handle(left_wheel_joint_, &pos[1], &vel[1], &eff[1]);
  joint_state_interface.registerHandle(state_left_wheel_handle);

  registerInterface(&joint_state_interface);

  hardware_interface::JointHandle vel_right_wheel_handle(joint_state_interface.getHandle(right_wheel_joint_), &cmd[0]);
  joint_vel_interface.registerHandle(vel_right_wheel_handle);

  hardware_interface::JointHandle vel_left_wheel_handle(joint_state_interface.getHandle(left_wheel_joint_), &cmd[1]);
  joint_vel_interface.registerHandle(vel_left_wheel_handle);

  registerInterface(&joint_vel_interface);

  nh.getParam("diff_drive_controller/wheel_radius", wheel_radius_);
  ROS_INFO("wheel_radius: %f", wheel_radius_);
}

void RaspberryPiMouseHW::read(ros::Duration d)
{
  pos[0] += vel[0] * d.nsec / 1000000000;
  vel[0] = cmd[0];
  pos[1] += vel[1] * d.nsec / 1000000000;
  vel[1] = cmd[1];
  // ROS_INFO("cmd=%u %f %f  %f %f", d.nsec, cmd[0], cmd[1], pos[0], pos[1]);
};

void RaspberryPiMouseHW::write()
{
  static int previous_left, previous_right;
  int left_freq, right_freq;
  std::ofstream ofs_left;
  std::ofstream ofs_right;
  if ((not ofs_left.is_open()) or (not ofs_right.is_open()))
  {
    ROS_ERROR("Cannot open /dev/rtmotor_raw_{l,r}0");
    return;
  }

  left_freq = (int)round(cmd[1] / (2.0 * M_PI * wheel_radius_ / 400.0) / 1000 * 24);
  right_freq = (int)round(cmd[0] / (2.0 * M_PI * wheel_radius_ / 400.0) / 1000 * 24);

  // ROS_INFO("left=%d right=%d %f ", left_freq, right_freq, wheel_radius_);

  // The buzzer and motor are using the common Raspberry Pi PWM function.
  // Write the PWM frequency only if there is new command value for the motor.
  if (previous_left != left_freq)
    ofs_left.open("/dev/rtmotor_raw_l0");
    ofs_left << left_freq << std::endl;
    ofs_left.close();
  if (previous_left != left_freq)
    ofs_right.open("/dev/rtmotor_raw_r0");
    ofs_right << right_freq << std::endl;
    ofs_right.close();
  previous_left = left_freq;
  previous_right = right_freq;
};