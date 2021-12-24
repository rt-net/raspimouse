# Raspberry Pi Mouse ROS package

![raspimouse](https://rt-net.jp/wp-content/uploads/2020/04/Raspberry-Pi-Mouse.png)

## Build Status

### master branch

[![industrial_ci](https://github.com/rt-net/raspimouse/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3774)
- Linux OS
  - Ubuntu Server 18.04/20.04
  - https://ubuntu.com/download/raspberry-pi
- Device Driver
  - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
- ROS
  - [Melodic Morenia](https://wiki.ros.org/melodic) / [Noetic Ninjemys](https://wiki.ros.org/noetic)

ROSとLinux OSは以下の組み合わせでのみ確認しています

* ROS Melodic + Ubuntu 18.04
* ROS Noetic + Ubuntu 20.04

## Installation

### Binary Insallation

準備中です

<!-- 
```sh
$ sudo apt install ros-$ROS_DISTRO-raspimouse
``` -->

### Source Build

```sh
# パッケージのダウンロード
$ cd ~/catkin_ws/src
$ git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse

# 依存パッケージのインストール
$ rosdep install -r -y -i --from-paths raspimouse

# もしraspimouse_descriptionパッケージがない場合は以下も実行
$ git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_description
$ rosdep install -r -y -i --from-paths raspimouse_description

# ビルド＆インストール
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

※raspimouse_ros_2と併用する場合、catkin_make時に`See documentation for policy CMP0002 for more details`というメッセージとともにエラーが出る場合があります。  
詳しくは[rt-net/raspimouse#1のコメント](https://github.com/rt-net/raspimouse/pull/1#issuecomment-990709564)を参照してください。

## QuickStart

```sh
# 端末 1
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch raspimouse_bringup raspimouse_robot.launch

# 端末 2
# モータの回転
$ source ~/catkin_ws/devel/setup.bash
$ rosservice call /motor_on
$ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}'
```

## Package Overview

### raspimouse_bringup

raspimouse_controlや関連パッケージを起動するためのlaunchファイルを揃えたROSパッケージです。

### raspimouse_control

Raspberry Pi Mouse制御用のROSパッケージです。  
[diff_drive_controller](https://wiki.ros.org/diff_drive_controller)に対応しています。

### raspimouse_msgs

raspimouse_controlで用いるROS Message定義ROSパッケージです。  
[2021年12月1日時点でのryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2/tree/9b62d996804b32b09108d1d7539e7386cee9f31d/msg)と互換性があります。

### raspimouse_stamped_msgs

[std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)の情報を付与したROS Message定義ROSパッケージです。  
[raspimouse_msgs/LightSensorValues](./raspimouse_msgs/msg/LightSensorValues.msg)などのセンサ情報の時刻を[tf/tfMessage](http://docs.ros.org/en/noetic/api/tf/html/msg/tfMessage.html)や[nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)などと同期する用途を想定しています。


## Topics

主要なトピックの一覧です。

### Subscribed

- `buzzer`

  Type: `std_msgs/Int16`

  ブザー駆動用トピックです。鳴らしたい音の周波数を指定します。

- `cmd_vel`

  Type: `geometry_msgs/Twist`

  モータ制御用トピックです。ロボットの進行方向の速度と旋回方向の角速度を指定します。

- `leds`

  Type: `raspimouse_msgs/LedValues`

  本体前方のLEDの制御用トピックです。4つのLEDの状態を指定します。

- `lightsensors`

  Type: `raspimouse_msgs/LightSensorValues`

  本体前方のLEDの制御用トピックです。4つのLEDの状態を指定します。


### Published

- `buttons`

  Type: `raspimouse_msgs/ButtonValues`

  本体上部のタクトスイッチのステータス配信用トピックです。

- `odom`

  Type: `nav_msgs/Odometry`

  本体のオドメトリ配信用トピックです。

## Services

主要なサービスの一覧です。

- `motor_on`

  Type: `std_srvs/Trigger`

  モータの電源をオンにする際にコールします。

- `motor_off`

  Type: `std_srvs/Trigger`

  モータの電源をオフにする際にコールします。

## Parameters

主要なパラメータの一覧です。

- `light_sensors/frequency`

  Type: `int`

  Default: `10`

  `lightsensors`トピックの配信周期を指定します。


[diff_drive_controller](https://wiki.ros.org/diff_drive_controller)のパラメータは[raspimouse_control/config/raspimouse_control.yaml](raspimouse_control/config/raspimouse_control.yaml)にて定義しています。  
パラメータの詳しい情報は[ROS wikiの解説](https://wiki.ros.org/diff_drive_controller)を参照してください。

- `diff_drive_controller/base_frame_id`

  Type: `string`

  `odom`と`tf`にてオドメトリの`child_frame`として設定するフレームの名前を指定します

- `diff_drive_controller/odom_frame_id`

  Type: `string`

  オドメトリを配信する際のフレームの名前を指定します。

- `diff_drive_controller/left_wheel`

  Type: `string`

  左の車輪のジョイント名を指定します。

- `diff_drive_controller/right_wheel`

  Type: `string`

  右の車輪のジョイント名を指定します。

- `diff_drive_controller/enable_odom_tf`

  Type: `bool`

  オドメトリをtfに配信するかどうかを指定します。

- `diff_drive_controller/wheel_radius`

  Type: `double`

  ロボットのホイールの半径（単位 mm）を指定します。

- `diff_drive_controller/wheel_separation`

  Type: `double`

  ロボットのホイールのトレッド（単位 mm）を指定します。

![](https://rt-net.github.io/images/raspberry-pi-mouse/raspimouse_control_rqt_graph.png)

## License

(C) 2020-2021 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。

### Acknowledgements

このソフトウェアはBSD 3-Clause Licenseで公開されている[ryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2)をベースに開発されています。

Copyright (c) 2017, Ryuichi Ueda

サードパーティのOSS利用についての詳しい情報は[OSSライセンスにもとづく表記](./THIRD-PARTY-LICENSE.md)を参照してください。
