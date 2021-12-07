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
 * This program is based on
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/src/leds.cpp
 * which is BSD 3-Clause "New" or "Revised" License.
 *
 * Copyright (c) 2017, Ryuichi Ueda
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "raspimouse_msgs/LedValues.h"

void output(std::ofstream* ofs, bool input)
{
  *ofs << (input ? '1' : '0') << std::endl;
}

void cb(const raspimouse_msgs::LedValues::ConstPtr& msg)
{
  std::ofstream ofs0("/dev/rtled0");
  output(&ofs0, msg->right_side);
  std::ofstream ofs1("/dev/rtled1");
  output(&ofs1, msg->right_forward);
  std::ofstream ofs2("/dev/rtled2");
  output(&ofs2, msg->left_forward);
  std::ofstream ofs3("/dev/rtled3");
  output(&ofs3, msg->left_side);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspimouse_leds_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("leds", 10, cb);

  ros::spin();
  exit(0);
}
