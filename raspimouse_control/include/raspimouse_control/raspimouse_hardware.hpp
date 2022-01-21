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
 */

#pragma once

#include <hardware_interface/joint_command_interface.h>  //VelocityJointInterface
#include <hardware_interface/joint_state_interface.h>    //JointStateInterface
#include <hardware_interface/robot_hw.h>

class RaspberryPiMouseHW : public hardware_interface::RobotHW
{
public:
  RaspberryPiMouseHW(ros::NodeHandle nh, ros::NodeHandle pnh);
  // ~RaspberryPiMouseHW();
  void read(ros::Duration d);
  void write();

private:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface joint_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  double wheel_radius_;
  std::string right_wheel_joint_ = "right_wheel_joint";
  std::string left_wheel_joint_ = "left_wheel_joint";
  std::string right_motor_device_file_ = "/dev/rtmotor_raw_r0";
  std::string left_motor_device_file_ = "/dev/rtmotor_raw_l0";
};
