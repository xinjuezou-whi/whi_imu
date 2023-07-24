# whi_imu
This package is the hardware driver of IMU for ROS. It currently supports the JY61/61P and JY901 of WIT Motion. Since it is a general IMU driver package and will be extended to support other IMU products, all specific product instances are derived from the base class ImuBase. Each product has its own parameters configuration file which would be included in its launch file.

## Prerequisites
The products relying on serial to communicate need serial package. This package leverages the serial package of ROS, so please first install it with the following commands:
```
sudo apt install ros-<ros distro>-serial
```

## Build Up
Go into your catkin workspace and initialize wstool if necessary (assuming ~/catkin_workspace as workspace path):
```
cd ~/catkin_workspace/src
git clone https://github.com/xinjuezou-whi/whi_imu.git
```

Build the package:
```
cd ..
catkin build
source ~/catkin_workspace/devel/setup.bash
```

## Parameters Configuration
Yaml file is used to bear the parameters including common and specified ones. Each product has its own configuration file. Below is an example of config file:
```
whi_imu:
  loop_hz: 50 # hz
  frame_id: 'imu'
  data_topic: 'imu_data'
  mag_topic: 'mag_data'
  temp_topic: 'temp_data'
  hardware_interface:
    module: 'JY61P'
    port: '/dev/imu'
    baudrate: 115200
    pack_length: 11
    # JY-61P is 0xff 0xaa 0x01 0x04 0x00 with unlock
    unlock: [0xff, 0xaa, 0x69, 0x88, 0xb5]
    reset_yaw: [0xff, 0xaa, 0x01, 0x04, 0x00]
    instruction_min_span: 5 # unit millisecond
    with_magnetic: true
    with_temperature: false
```

There are three major parts that need your attention:
The first is to configure the frame ID to meet your TF tree:
- frame_id: the frame ID of the IMU in your TF tree 

And second, configure the topic params to meet your subscriber:
- data_topic: the orientation data
- mag_topic: the magnetic data
- temp_topic: the temperature data

Then the last, configure the serial communication params to meet your IMU's setting:
- device port: the serial port of your IMU
- baudrate: the serial baudrate of your IMU

This configuration file should be included in the launch file. Update the name of yaml file below the line of "params" with one of the corresponding products, like the bellowing example the product is specified as JY61P: 
```
  <!-- params -->
  <rosparam file="$(find whi_imu)/config/imu_hardware_jy61p.yaml" command="load"/>
```

## Service
It advertises the service "imu_reset" for resetting the yaw, which is helpful at the stage of full initialization

## Run
Launch the whi_imu node with commands:
```
cd ~/catkin_workspace/
roslaunch whi_imu whi_imu.launch
```
Or with a reset flag to set yaw as zero:
```
roslaunch whi_imu whi_imu.launch reset:=true
```

Then in another terminal, use the rostopic command to check its outputs:
```
rostopic echo /imu_data
```

![imu](https://user-images.githubusercontent.com/72239958/205224541-0f30e5e7-d1aa-4db5-af34-10e4fe3ce7f2.gif)
