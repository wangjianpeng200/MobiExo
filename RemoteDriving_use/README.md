# **Seed**
请仔细阅读README以及开发指南
## 系统总览
![overview](doc/IntelligentDrive.png)

## 安装向导
推荐在Ubuntu16.04下使用<br>
1.安装ROS Kinetic桌面完整版,Kinetic为Ubuntu 16.04下的ROS版本名称，若当前使用其他版本的Ubuntu，请注意选择相应版本<br>
请按照官方教程安装：[http://wiki.ros.org/cn/kinetic/Installation/Ubuntu](http://wiki.ros.org/cn/kinetic/Installation/Ubuntu)     
2.安装依赖
```shell
sudo apt-get install libpcap-dev  
sudo apt-get install ros-$ROS_DISTRO-tf2    
sudo apt-get install ros-$ROS_DISTRO-geographic-msgs
sudo apt-get install ros-$ROS_DISTRO-tf2-sensor-msgs
```
若要编译图像相关部分还需安装<br>
相机驱动Spinnaker<br>
caffe-ssd<br>
或者切换代码分支到except_vision，该分支中移除了图像相关部分的代码
```shell
git checkout except_vision
```
3.添加工程目录到环境变量
```shell
export SEED_HOME=放置代码的目录/Seed
```
example:
```shell
export SEED_HOME=/home/sushold/Desktop/workspace/Seed
```
该语句仅对当前终端有效，若要永久生效可将此语句添加到~/.bashrc中。  
4.编译
```shell
cd $SEED_HOME
catkin_make
```

#### ubuntu下c++连接postgresql数据库
```
法1：sudo apt-get install libpqxx-dev
```
法2：
```
wget http://pqxx.org/download/software/libpqxx/libpqxx-4.0.tar.gz
tar xvfz libpqxx-4.0.tar.gz
cd libpqxx-4.0
./configure
make
sudo make install
```
