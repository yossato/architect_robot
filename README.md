
# 4th Robot Package [![Build Status](https://travis-ci.org/CIR-KIT/fourth_robot_pkg.svg?branch)](https://travis-ci.org/CIR-KIT/fourth_robot_pkg)

### Bring Up
##### 1. Ethernetの設定を行う
IP      : `192.168.10.100`  
Mask    : 24  
Gateway : `192.168.10.1`  

##### 2. Path通し
```bash
source <catkin_ws>/devel/setup.bash
```

##### 3. urbtcを有効に
```bash
source <fourth_robot_driver>/script/module.sh
```

#### ラジコンモードの場合
###### 1. Joy Stick を接続
`sudo sixad --start` を実行した後，PSボタンを押下でBluetooth接続
###### 2. launch
```bash
roslaunch fourth_robot_bringup joy_control.launch
```

#### マッピングモードの場合
###### 1. Joy Stick を接続
`sudo sixad --start` を実行した後，PSボタンを押下でBluetooth接続
###### 2. launch
```bash
roslaunch fourth_robot_bringup 2dmapping.launch
```

# architect_robot
Line tracer robot with gazebo and ROS.

