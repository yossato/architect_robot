current_dir=$(pwd)
setup_bash_dir="$HOME/Documents/catkin_ws/src/TC2015/fourth_robot/script"
iMCs_conf_name="iMCs01.yaml"

if [ -z "$1" ]; then
	echo ""
	echo "The iMCs01 config file name is \"$iMCs_conf_name\" ."
else
	iMCs_conf_name=$1
	echo ""
	echo "The iMCs01 config file name is \"$iMCs_conf_name\" ."
fi

# load urbtc module
cd ${setup_bash_dir}
source ../fourth_robot_driver/script/module.bash

# set ROS_IP
cd ${setup_bash_dir}
source ./set_IP_server.bash
echo ""
env | grep ROS_

urbtc_name=$(ls /dev/urbtc*)
gim30_name=$(ls /dev/ttyUSB*)

if [ -z "$urbtc_name" ]; then
	echo "/dev/urbtc* : no such file or directory"
else
	sudo chmod 777 /dev/urbtc*
	cd ${setup_bash_dir}
	yaml_file="../fourth_robot_driver/config/$iMCs_conf_name"
	cat ${yaml_file} | sed -i -e "s|/dev/urbtc.|$urbtc_name|g" ${yaml_file}
fi

if [ -z "$gim30_name" ]; then
	echo "/dev/ttyUSB* : no shuch file or directory"
else
	sudo chmod 777 /dev/ttyUSB*
fi

cd ${current_dir}
