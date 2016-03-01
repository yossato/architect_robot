module_bash_dir=$(dirname ${BASH_SOURCE:-$0})
cd ${module_bash_dir}
cd ..
sudo rmmod urbtc.ko
sudo insmod include/iMCs01_driver/urbtc.ko 
