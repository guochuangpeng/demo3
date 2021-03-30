### Documentation

### Dependencies


### Install

When you run this package first time , you need to initial the serial device,get the permission to read the serial port
 and bind the serial port name in to a fix-name by add a dev rule. You should do the following steps.

#### step 0:
When use catkin build, sometime can't find the lib of serial, try to build & install the serial package

follow the serial package README, copy the compile lib file to the /usr/lib .

#### step 1:

Open the terminal and input: 

"sudo su" to get the root permission 

type "bash ./init_serial.sh" to setting the serial automatically.

#### step 2:

set the mavros setting.First you need to install mavros like:

TODO

If you install mavros by apt-get ,make sure mavros pluginlists don't set vision_pose in backlist.

The setting file is /opt/ros/kinetic/share/mavros/launch/px4_pluginlists.yaml

setting the px4 serial communication rate in /opt/ros/kinetic/share/mavros/launch/px4_pluginlists.yaml

#### step 3:

make sure you have install ros. Open the terminal and input:

gedit ~/.bashrc insert in the end of file:

```
source /opt/ros/kinetic/setup.bash
source ~/uwb_ws/devel/setup.bash
```

reopen the terminal

Set anchor position in ./launch/localization.launch

>run : **roslaunch mavros px4.launch** first to start the mavros for pixhawk.

>run : **roslaunch uwb localization.launch** to get the postion from uwb.

**Note** :Power on the anchor in the order of 0, 1, 2, 3 when you want to test the device.

### Authors

Gavin

### Contact

Xue.JW@qq.com


