current_dir=$(pwd)
setup_bash_dir="$HOME/Documents/catkin_ws/src/TC2015/fourth_robot/script"

# set ROS_IP
cd ${setup_bash_dir}
source ./set_IP_client.bash
echo ""
env | grep ROS_

cd ${current_dir}
