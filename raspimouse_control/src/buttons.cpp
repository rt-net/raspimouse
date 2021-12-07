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
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/src/buttons.cpp
 * which is BSD 3-Clause "New" or "Revised" License.
 *
 * Copyright (c) 2017, Ryuichi Ueda
 * https://github.com/ryuichiueda/raspimouse_ros_2/blob/9b62d996804b32b09108d1d7539e7386cee9f31d/LICENSE
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "raspimouse_msgs/ButtonValues.h"

bool readButton(const char* name)
{
  std::ifstream ifs(name);
  char c;
  ifs >> c;
  return c == '0';
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "buttons");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<raspimouse_msgs::ButtonValues>("buttons", 5);

  ros::Rate loop_rate(10);
  raspimouse_msgs::ButtonValues msg;
  int c[3] = { 0, 0, 0 };
  while (ros::ok())
  {
    msg.front = readButton("/dev/rtswitch0");
    msg.mid = readButton("/dev/rtswitch1");
    msg.rear = readButton("/dev/rtswitch2");

    c[0] = msg.front ? 1 + c[0] : 0;
    c[1] = msg.mid ? 1 + c[1] : 0;
    c[2] = msg.rear ? 1 + c[2] : 0;

    if (c[0] > 4)
    {
      msg.front_toggle = not msg.front_toggle;
      c[0] = 0;
    }
    if (c[1] > 4)
    {
      msg.mid_toggle = not msg.mid_toggle;
      c[1] = 0;
    }
    if (c[2] > 4)
    {
      msg.rear_toggle = not msg.rear_toggle;
      c[2] = 0;
    }

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  exit(0);
}