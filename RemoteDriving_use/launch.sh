#! /bin/bash
#sudo ptpd -M -i eno1 -C
echo "nvidia" |
sudo -S chmod 777 /dev/ttyUSB*
export SEED_HOME=`pwd`
source devel/setup.bash
roslaunch ${SEED_HOME}/src/example.launch

