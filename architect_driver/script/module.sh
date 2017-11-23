file=`rospack find architect_driver`/include/iMCs01_driver/urbtc.ko
sudo rmmod urbtc.ko
sudo insmod ${file}
