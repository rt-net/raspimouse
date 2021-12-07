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

  hardware_interface::JointStateHandle state_right_wheel_handle("right_wheel_joint", &pos[0], &vel[0], &eff[0]);
  joint_state_interface.registerHandle(state_right_wheel_handle);

  hardware_interface::JointStateHandle state_left_wheel_handle("left_wheel_joint", &pos[1], &vel[1], &eff[1]);
  joint_state_interface.registerHandle(state_left_wheel_handle);

  registerInterface(&joint_state_interface);

  hardware_interface::JointHandle vel_right_wheel_handle(joint_state_interface.getHandle("right_wheel_joint"), &cmd[0]);
  joint_vel_interface.registerHandle(vel_right_wheel_handle);

  hardware_interface::JointHandle vel_left_wheel_handle(joint_state_interface.getHandle("left_wheel_joint"), &cmd[1]);
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
  int left, right;
  std::ofstream ofsL("/dev/rtmotor_raw_l0");
  std::ofstream ofsR("/dev/rtmotor_raw_r0");
  if ((not ofsL.is_open()) or (not ofsR.is_open()))
  {
    ROS_ERROR("Cannot open /dev/rtmotor_raw_{l,r}0");
    return;
  }

  left = (int)round(cmd[1] / (2.0 * 3.14159 * wheel_radius_ / 400.0) / 1000 * 24);
  right = (int)round(cmd[0] / (2.0 * 3.14159 * wheel_radius_ / 400.0) / 1000 * 24);
  // ROS_INFO("left=%d right=%d %f ", left, right, wheel_radius_);
  ofsL << left << std::endl;
  ofsR << right << std::endl;
};