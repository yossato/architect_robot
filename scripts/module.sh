file=`rospack find fourth_robot_driver`/include/iMCs01_driver/urbtc.ko
sudo rmmod urbtc.ko
sudo insmod ${file}
