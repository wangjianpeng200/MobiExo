#! /bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400 
sudo busybox devmem 0x0c303008 32 0x0000C458 
sudo busybox devmem 0x0c303010 32 0x0000C400 
sudo busybox devmem 0x0c303018 32 0x0000C458

sudo busybox devmem 0x0c303000	# 0x0000C400
sudo busybox devmem 0x0c303008	# 0x0000C458
sudo busybox devmem 0x0c303010	# 0x0000C400
sudo busybox devmem 0x0c303018	# 0x0000C458

sudo modprobe can          #Insert CAN BUS subsystem support module.
sudo modprobe can_raw      #Insert Raw CAN protocol module (CAN-ID filtering)
sudo modprobe mttcan       #Real CAN interface support
sudo modprobe can_dev

#set can interface 
sudo ip link set can0 type can bitrate 500000     #设置波特率500k
sudo ip link set can1 type can bitrate 500000     #设置波特率500k

# 回环-测试时候用
#sudo ip link set can0 type can bitrate 500000 loopback on
#sudo ip link set can1 type can bitrate 500000 loopback on

sudo ip link set up can0                          #开启can0
sudo ip link set up can1                          #开启can1

#cansend can0 123#1111111111111111
