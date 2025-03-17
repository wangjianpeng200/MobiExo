#! /bin/bash
#sudo ptpd -M -i eno1 -C

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.bash
echo "nvidia" | 
sudo -S chmod 777 /dev/ttyUSB*
source ${SEED_HOME}/devel/setup.bash 
roslaunch ${SEED_HOME}/src/example.launch 

