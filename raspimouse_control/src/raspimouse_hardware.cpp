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

const static int RIGHT= 0;
const static int LEFT = 1;

RaspberryPiMouseHW::RaspberryPiMouseHW(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  std::fill_n(pos, 2, 0.0);
  std::fill_n(vel, 2, 0.0);
  std::fill_n(eff, 2, 0.0);
  nh.getParam("diff_drive_controller/right_wheel", right_wheel_joint_);
  nh.getParam("diff_drive_controller/left_wheel", left_wheel_joint_);
  pnh.getParam("device_file_right_motor", right_motor_device_file_);
  pnh.getParam("device_file_left_motor", left_motor_device_file_);
  ROS_INFO("right_wheel_joint: %s, left_wheel_joint: %s", right_wheel_joint_.c_str(), left_wheel_joint_.c_str());

  hardware_interface::JointStateHandle state_right_wheel_handle(right_wheel_joint_, &pos[RIGHT], &vel[RIGHT], &eff[RIGHT]);
  joint_state_interface.registerHandle(state_right_wheel_handle);

  hardware_interface::JointStateHandle state_left_wheel_handle(left_wheel_joint_, &pos[LEFT], &vel[LEFT], &eff[LEFT]);
  joint_state_interface.registerHandle(state_left_wheel_handle);

  registerInterface(&joint_state_interface);

  hardware_interface::JointHandle vel_right_wheel_handle(joint_state_interface.getHandle(right_wheel_joint_), &cmd[RIGHT]);
  joint_vel_interface.registerHandle(vel_right_wheel_handle);

  hardware_interface::JointHandle vel_left_wheel_handle(joint_state_interface.getHandle(left_wheel_joint_), &cmd[LEFT]);
  joint_vel_interface.registerHandle(vel_left_wheel_handle);

  registerInterface(&joint_vel_interface);

  nh.getParam("diff_drive_controller/wheel_radius", wheel_radius_);
  ROS_INFO("wheel_radius: %f", wheel_radius_);
}

RaspberryPiMouseHW::~RaspberryPiMouseHW()
{
  std::ofstream ofs_left;
  std::ofstream ofs_right;

  ofs_left.open(left_motor_device_file_);
  if (not ofs_left.is_open())
  {
    ROS_ERROR("Cannot open %s", left_motor_device_file_.c_str());
    return;
  }
  ofs_left << 0 << std::endl;
  ofs_left.close();

  ofs_right.open(right_motor_device_file_);
  if (not ofs_right.is_open())
  {
    ROS_ERROR("Cannot open %s", right_motor_device_file_.c_str());
    return;
  }
  ofs_right << 0 << std::endl;
  ofs_right.close();
}

void RaspberryPiMouseHW::read(ros::Duration d)
{
  pos[RIGHT] += vel[RIGHT] * d.nsec / 1000000000;
  vel[RIGHT] = cmd[RIGHT];
  pos[LEFT] += vel[LEFT] * d.nsec / 1000000000;
  vel[LEFT] = cmd[LEFT];
  // ROS_INFO("cmd=%u %f %f  %f %f", d.nsec, cmd[RIGHT], cmd[LEFT], pos[RIGHT], pos[LEFT]);
};

void RaspberryPiMouseHW::write()
{
  static int previous_left, previous_right;
  int left_freq, right_freq;
  std::ofstream ofs_left;
  std::ofstream ofs_right;

  // cmd[] is given as wheel angular velocity (rad/sec)
  // motor : 400 pluse/rotate
  // cmd / (2.0 * M_PI) : taget speed (rotate/sec)
  // cmd / (2.0 * M_PI) * 400 : taget speed (pulse/sec)
  left_freq = (int)round(cmd[LEFT] / (2.0 * M_PI) * 400.0);
  right_freq = (int)round(cmd[RIGHT] / (2.0 * M_PI) * 400.0);

  // ROS_INFO("left=%d right=%d %f ", left_freq, right_freq, wheel_radius_);

  // The buzzer and motor are using the common Raspberry Pi PWM function.
  // Write the PWM frequency only if there is new command value for the motor.
  // Raspberry PiのPWM機能はブザーとモータで共通に使用される。
  // そのため、モータの指示値がない場合は、PWM周波数を書き込まない。
  if (previous_left != left_freq)
  {
    ofs_left.open(left_motor_device_file_);
    if (not ofs_left.is_open())
    {
      ROS_ERROR("Cannot open %s", left_motor_device_file_.c_str());
      return;
    }
    ofs_left << left_freq << std::endl;
    ofs_left.close();
  }

  if (previous_left != left_freq)
  {
    ofs_right.open(right_motor_device_file_);
    if (not ofs_right.is_open())
    {
      ROS_ERROR("Cannot open %s", right_motor_device_file_.c_str());
      return;
    }
    ofs_right << right_freq << std::endl;
    ofs_right.close();
  }
  previous_left = left_freq;
  previous_right = right_freq;
};