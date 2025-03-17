#! /bin/bash
#gnome-terminal -- rosclean purge -y
sleep 2
gnome-terminal -- bash /home/nvidia/Seed_new/once.sh
sleep 2
gnome-terminal -- bash /home/nvidia/Seed_new/launch.sh
sleep 10
gnome-terminal -- bash /home/nvidia/Seed_new/vehicle_control.sh


