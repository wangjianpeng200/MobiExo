#!/bin/bash
#sudo ifconfig can0 down
#sudo ifconfig can1 down
sudo modprobe can
sleep 0.2
sudo modprobe can_raw
sleep 0.2
sudo modprobe mttcan


sleep 0.2
sudo ip link set can0 down 
sleep 0.2
sudo ip link set can1 down 
sleep 0.2
sudo ip link set can2 down 
sleep 0.2
sudo ip link set can3 down 

sleep 0.2
sudo ip link set can0 type can bitrate 500000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on restart-ms 100
sleep 0.2
sudo ip link set can1 type can bitrate 500000 sample-point 0.8 dbitrate 5000000 dsample-point 0.8 fd on restart-ms 100
sleep 0.2
sudo ip link set can2 type can bitrate 500000 sample-point 0.8 dbitrate 5000000 dsample-point 0.8 fd on restart-ms 100
sleep 0.2
sudo ip link set can3 type can bitrate 500000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on restart-ms 100


sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can2 txqueuelen 1000
sudo ip link set can3 txqueuelen 1000

sudo ifconfig can0 up
sudo ifconfig can1 up
sudo ifconfig can2 up
sudo ifconfig can3 up


candump -x any &
