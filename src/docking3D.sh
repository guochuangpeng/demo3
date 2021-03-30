#!/bin/sh
# Please run with sudo.
# This bash will launch all the components of the system, in the order of:
#sudo -s
#sudo rm -rf /root/.ros/log
#su nvidia 

user_name=nvidia

cd /home/nvidia/catkin_ws_docking

rosclean purge -y

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws_docking/devel/setup.bash

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/nvidia/catkin_ws_docking/
#export ROS_HOSTNAME=localhost
export ROS_MASTER_URI="http://nvidia-desktop:11311"
#export ROS_MASTER_URI="http://xupengfei-Lenovo-Erazer-Y50-70:11311"
export ROS_HOME=/home/nvidia/catkin_ws_docking/logfiles

#export ROS_HOME=/home/nvidia/catkin_ws/logfiles
#export ROS_HOSTNAME=nvidia-10
#export ROS_MASTER_URI=http://192.168.1.100:11311

# kill all screen session 
pkill screen
# delay between launching various modules
module_delay=10





# enable serial ports
sudo chmod a+rw /dev/px4_01
#sudo chmod a+rw /dev/ttyUSB1
#sudo chmod a+rw /dev/ttyACM0

echo "The system is booting..."

##roscore
screen -d -m -S roscore bash -c "source devel/setup.bash; roscore; exec bash -i"
sleep 10
echo "roscore ready."


# --------------------------------------------
# prepareation

#source devel/setup.bash
# 0. px4
screen -d -m -S mavros bash -c "source /opt/ros/melodic/setup.bash; roslaunch mavros px4.launch; exec bash -i"
sleep ${module_delay}
echo "mavros ready."

#source devel/setup.bash
# 1. uav_state
#screen -d -m -S uav_state bash -c "source install/setup.bash; roslaunch uav_state uav_state.launch; exec bash -i"
#sleep ${module_delay}
#echo "uav_state ready."

#source devel/setup.bash
# 2. ref_generation
#screen -d -m -S ref_generation bash -c "source install/setup.bash; roslaunch ref_generation ref_generation.launch; exec bash -i"
#sleep ${module_delay}
#echo "ref_generation ready."

# 3. uwb_01
screen -d -m -S uwb_01 bash -c "source devel/setup.bash; roslaunch uwb uwb.launch; exec bash -i"
#sleep ${module_delay}
echo "uwb_01 ready."

# 3. relative_localization
screen -d -m -S relative_localization bash -c "source devel/setup.bash; roslaunch relative_localization RelativeLocalization3D.launch; exec bash -i"
sleep ${module_delay}
echo "relative_localization ready."

# 3. movement
screen -d -m -S movement bash -c "source devel/setup.bash; roslaunch movement movement3D.launch; exec bash -i"
#sleep ${module_delay}
echo "movement ready."
#source devel/setup.bash




# 4. mission
#screen -d -m -S mission bash -c "source devel/setup.bash; roslaunch mission TaskManager.launch; exec bash -i"
#sleep ${module_delay}
#echo "mission ready."



screen -d -m -S vrpn bash -c "source /opt/ros/kinetic/setup.bash; roslaunch vrpn_client_ros sample.launch server:=192.168.2.9
#; exec bash -i"
#sleep ${module_delay}
echo "vrpn ready."

dat_file=$(date +"%y%m%d_%H%M%S")
mkdir -p /home/nvidia/catkin_ws_docking/log/${dat_file}


#nohup rostopic echo -p /uwb_info                        >/home/nvidia/catkin_ws_docking/log/${dat_file}/uwb.txt 2>&1 &
nohup rostopic echo -p /relative_position/uwb/pos_result                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/uwb_pos_result.txt 2>&1 &
nohup rostopic echo -p /relative_position/visual/pos_result                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/visual_pos_result.txt 2>&1 &
nohup rostopic echo -p /mavros/setpoint_velocity/cmd_vel_unstamped                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/vel_out.txt 2>&1 &
nohup rostopic echo -p /mavros/setpoint_position/local                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/pos_out.txt 2>&1 &
nohup rostopic echo -p /uwb_raw_range                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/raw_range.txt 2>&1 &
nohup rostopic echo -p /uwb_kf_range                   >/home/nvidia/catkin_ws_docking/log/${dat_file}/kf_range.txt 2>&1 &
nohup rostopic echo -p /acc2pose/matlab                 >/home/nvidia/catkin_ws_docking/log/${dat_file}/dis_time.txt 2>&1 &
nohup rostopic echo -p /mavros/imu/data                 >/home/nvidia/catkin_ws_docking/log/${dat_file}/imu_data.txt 2>&1 &
nohup rostopic echo -p /mavros/setpoint_raw/local       >/home/nvidia/catkin_ws_docking/log/${dat_file}/setpoit.txt 2>&1 &
nohup rostopic echo -p /mavros/local_position/pose      >/home/nvidia/catkin_ws_docking/log/${dat_file}/local_position.txt 2>&1 &
nohup rostopic echo -p /mavros/local_position/velocity_local      >/home/nvidia/catkin_ws_docking/log/${dat_file}/local_pose.txt 2>&1 &
nohup rostopic echo -p /waypoint                        >/home/nvidia/catkin_ws_docking/log/${dat_file}/waypoint.txt 2>&1 &
nohup rostopic echo -p /mavros/distance_sensor/hrlv_ez4_pub                         >/home/nvidia/catkin_ws_docking/log/${dat_file}/lidar_alt.txt 2>&1 &
#source devel/setup.bash
# 4. img_reader
#screen -d -m -S img_reader bash -c "source devel/setup.bash; roslaunch img_reader image_only.launch; exec bash -i"
#sleep ${module_delay}
#echo "img_reader ready."

# 5. multimaster for uav communication
# master_discovery_fkie
#source devel/setup.bash
#screen -d -m -S master_discovery_fkie bash -c "source devel/setup.bash; roslaunch master_discovery_fkie master_discovery_contest.launch; exec bash -i"
#echo "master_discovery_fkie ready."

#source devel/setup.bash
#screen -d -m -S master_sync_fkie bash -c "source devel/setup.bash; roslaunch master_sync_fkie master_sync_contest.launch; exec bash -i"
#echo "master_sync_fkie ready."
#sleep ${module_delay}

echo "System is started."
