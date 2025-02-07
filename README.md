<h1 align="center"> ACE: A Cross-platform Visual-Exoskeletons
for Low-Cost Dexterous Teleoperation </h1>

<p align="center">
    <a href="https://aaronyang1223.github.io/" style="font-size: 20px;">
        Shiqi Yang
    </a>
    ·
    <a href="https://minghuanliu.com" style="font-size: 20px;">
        Minghuan Liu
    </a>
    ·
    <a href="https://yzqin.github.io" style="font-size: 20px;">
        Yuzhe Qin
    </a>
    ·
    <a href="https://dingry.github.io" style="font-size: 20px;">
        Runyu Ding
    </a>
    ·
    <a href="https://ace-teleop.github.io" style="font-size: 20px;">
        Jialong Li
    </a>
    <br>
    <a href="https://chengxuxin.github.io" style="font-size: 20px;">
        Xuxin Cheng
    </a>
    ·
    <a href="https://rchalyang.github.io" style="font-size: 20px;">
        Ruihan Yang
    </a>
    ·
    <a href="https://www.cs.cmu.edu/~shayi/" style="font-size: 20px;">
        Sha Yi
    </a>
    ·
    <a href="https://xiaolonw.github.io" style="font-size: 20px;">
        Xiaolong Wang
    </a>
</p>

<p align="center">
    <img src="./src/UCSanDiegoLogo-BlueGold.png" height="50">
</p>

<h3 align="center"> CoRL 2024 </h3>

<p align="center">
<h3 align="center"><a href="https://ace-teleop.github.io/">Website</a> | <a href="http://arxiv.org/abs/2408.11805">arXiv</a> | <a href="https://github.com/ACETeleop/ACE_hardware">Hardware</a> </h3>
  <div align="center"></div>
</p>

<p align="center">
<img src="./src/sim_demo.webp" width="80%"/>
</p>

## 介绍
该仓库包含了 ACE 的所有软件，主要包括三个组件：服务器、控制器和仿真。此外，我们还提供了用于设置硬件的实用工具。结合 ACE 硬件，你可以在仿真环境中快速使用各种末端执行器和机器人进行遥操作，或者使用控制器发出的指令在现实世界中操作机器人。有关 STL 文件和硬件搭建说明，请参阅 ACE 硬件仓库 ACE 以自行构建。
# 关键组件
服务器： 接收手部图像和关节角度作为输入，并在映射后输出手腕姿态和手部关键点。
控制器： 处理来自服务器的输入，并使用逆运动学（IK）生成控制指令。
仿真： 接收指令并在仿真环境中可视化操作。
# 创建虚拟环境
使用python3.8环境 只能使用仿真控制，无法控制真实机械臂。使用python 3.9环境只能控制真实机械臂，无法仿真控制。建议同时安装两种python的虚拟环境。
```bash
  pip install -e .
  pip install Robotic_Arm
```

## 电机组装与校准
# 更新电机 ID
安装 dynamixel_wizard。
默认情况下，每个电机的 ID 为 1。为了使多个动力舵机能由同一个 U2D2 控制器板控制，每个动力舵机必须有一个唯一的 ID。ACE 有六个舵机，从基座到手腕，它们的 ID 应依次设置为 1 到 6。这个过程必须逐个电机进行。
# 步骤：
1.将单个电机连接到控制器，并将控制器连接到计算机。
2.打开 dynamixel wizard 并点击扫描以连接到电机。
3.对于每个舵机，将电机 ID 地址从 1 更改为目标 ID（1 到 6）。
4.对每个电机重复此过程，依次分配从 1 到 6 的唯一 ID。

下载```DYNAMIXEL Wizard```软件，[下载链接在此处](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)，**将一个舵机单独直接连上板子**

```DYNAMIXEL Wizard```软件安装后打开：

![](./images/image-3.png)

点击图左上方齿轮图标，设置扫描参数如下：

![](./images/image-4.png)

点击图左上方的放大镜图标进行扫描：

![](./images/image-5.png)

根据接入顺序，可能是`/dev/ttyUSB0`或`/dev/ttyUSB1`，若扫描到结果如上图即成功。此窗口会自动关闭。

点击中间如图所示：

![](./images/image-6.png)

点击右侧栏ID选项，可设置ID。

![](./images/image-7.png)

向下滑动滚轮，可看到`Save`，保存即可。
依次操作，将两个机械臂分别进行1到6编号。
最后两个机械臂接上，扫描一下：

![](./images/image-8.png)

若得到如图所示效果，则配置成功。

# 获取偏移量
设置好电机 ID 后，你就可以连接到 ACE 硬件了。然而，每个电机都有其自身的关节偏移量，这会导致你的 ACE 硬件与 ACE URDF 之间存在差异。动力舵机具有对称的四孔图案，因此关节偏移量是 π/2 的倍数。为了解决这个问题，你可以使用以下代码来校准你的 ACE 硬件。

通过运行```ls /dev/serial/by-id```并查找以```usb-FTDI_USB**-**Serial_Converter ```开头的路径（在 Ubuntu 上）来找到你的 U2D2 Dynamixel 设备的端口 ID。
将 ACE 硬件设置为下图所示的姿势，然后运行代码

![](./images/image-13.png)

```python
  python3 -m ace_teleop.dynamixel.calibration.get_offset --port /dev/serial/by-id/usb-FTDI_USB**-**Serial_Converter_FT8J0QI3-if00-port0 --type left
```

# 姿态校准
获取偏移量后，进入 ace_teleop/dynamixel/calibration/config.py 并在 PORT_CONFIG_MAP 中添加一个 DynamixelRobotConfig。注意：你只需从文件中复制一个现有配置，并根据需要修改 port 和 joint_offsets 即可。
测试校准
校准完成后，你可以使用以下命令测试结果。你应该看到现实世界中的 ACE 硬件的行为与仿真中的行为相同。
```python
  python -m ace_teleop.dynamixel.calibration.test_calibration --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0QI3-if00-port0 --type left
```
# 注意：test_calibration 需要 sapien==3.0.0b0，这可能与 dex_retargeting==0.1.1 冲突。
# 建议创建一个单独的环境以避免潜在错误。

## 摄像头测试
查找摄像头索引并测试摄像头帧率
此代码将查找所有网络摄像头并显示帧率。
```python
  python3 -m ace_teleop.tools.find_webcam
```
获取摄像头索引后，进入 ace_teleop/configs/server 并更改相应 yml 文件中的 cam_num。

## 真实机械臂ip修改
请使用以太网线连接机械臂控制器和电脑，将以电脑太网口的IPV4配置为192.169.1.10。打开浏览器，若使用有线连接，则网址输入 192.168.1.18 进入登录页。若可进入登录界面，则连接正常。

![alt text](images/image-9.png)

在睿尔曼的机械臂操作界面内，将右臂的有线连接ip改为```192.168.1.19```

## 测试与运行

# 仿真测试
在python 3.8环境下进行仿真测试，测试穿戴设备是否能够正确控制仿真机械臂
在一个终端中运行服务端
```python
  python scripts/start_server.py --config franka_gripper
```
在一个终端中运行仿真

```python
  python scripts/teleop_sim.py --config franka
```

# 真实机械臂测试
在python 3.9环境下进行仿真测试，测试穿戴设备是否能够正确控制仿真机械臂
在一个终端中运行服务端
```python
  python scripts/start_server.py --config franka_gripper
```
在一个终端中运行仿真

```python
  python scripts/teleop_cmd.py --config franka
```


系统图
image
该系统易于使用，只需三个步骤即可控制各种末端执行器和机器人。
校准硬件： 按照说明 校准 硬件。
开始遥操作：
启动服务器。
如果你需要在 仿真环境 中控制机器人，只需运行 teleop_sim.py。
如果你需要控制 真实机器人，修改 teleop_cmd 并将其与真实机器人控制代码连接。
注意：
遥操作系统将自动映射旋转和位置，然后在 初始化 后启动。初始化过程在双手保持静止且手指平展时开始。
服务器和控制器需要相应的配置。
ACE 服务器（从 ACE 硬件或键盘获取输入）
启动服务器
如果你只是想看看代码是如何工作的，可以使用键盘进行操作：
python
python3 scripts/start_server.py --config h1_inspire --keyboard
在运行 teleop_sim 代码（见下文）后，你可以使用 wasd 控制左手腕，使用 ↑←↓→ 控制右手腕。注意：控制信号是从服务器端发送的，因此前台窗口必须是运行上述脚本的终端。
如果你有一套 ACE 遥操作硬件，可以尝试以下操作：
python

# 普通模式

python3 scripts/start_server.py --config h1_inspire

# 镜像模式

python3 scripts/start_server.py --config h1_inspire_mirror
ACE 控制器（从服务器获取输入）
运行控制器
python

# 用于获取指令

python scripts/teleop_cmd.py --config h1_inspire
仿真（从控制器获取输入）
在仿真环境中运行
我们提供了在仿真环境中控制机器人的代码。在控制真实机器人之前，你可以轻松使用仿真来测试配置。
python

# 如果你想运行仿真示例，请安装 isaacgym。

python scripts/teleop_sim.py --config h1_inspire
校准
更新电机 ID
安装 dynamixel_wizard。
默认情况下，每个电机的 ID 为 1。为了使多个动力舵机能由同一个 U2D2 控制器板控制，每个动力舵机必须有一个唯一的 ID。ACE 有六个舵机，从基座到手腕，它们的 ID 应依次设置为 1 到 6。这个过程必须逐个电机进行。
步骤：
将单个电机连接到控制器，并将控制器连接到计算机。
打开 dynamixel wizard 并点击扫描以连接到电机。
对于每个舵机，将电机 ID 地址从 1 更改为目标 ID（1 到 6）。
对每个电机重复此过程，依次分配从 1 到 6 的唯一 ID。
获取偏移量
设置好电机 ID 后，你就可以连接到 ACE 硬件了。然而，每个电机都有其自身的关节偏移量，这会导致你的 ACE 硬件与 ACE URDF 之间存在差异。动力舵机具有对称的四孔图案，因此关节偏移量是 π/2 的倍数。为了解决这个问题，你可以使用以下代码来校准你的 ACE 硬件。
端口： 通过运行 ls /dev/serial/by-id 并查找以 usb-FTDI_USB**-**Serial_Converter 开头的路径（在 Ubuntu 上）来找到你的 U2D2 Dynamixel 设备的端口 ID。在 Mac 上，在 /dev/ 中查找以 cu.usbserial 开头的设备。
类型： 指定你要校准的手臂。
python

# 将 ACE 硬件设置为下图所示的姿势，然后运行代码

python3 -m ace_teleop.dynamixel.calibration.get_offset --port /dev/serial/by-id/usb-FTDI_USB**-**Serial_Converter_FT8J0QI3-if00-port0 --type left

校准姿势
image
获取偏移量后，进入 ace_teleop/dynamixel/calibration/config.py 并在 PORT_CONFIG_MAP 中添加一个 DynamixelRobotConfig。注意：你只需从文件中复制一个现有配置，并根据需要修改 port 和 joint_offsets 即可。
测试校准
校准完成后，你可以使用以下命令测试结果。你应该看到现实世界中的 ACE 硬件的行为与仿真中的行为相同。
python

# 注意：test_calibration 需要 sapien==3.0.0b0，这可能与 dex_retargeting==0.1.1 冲突。

# 建议创建一个单独的环境以避免潜在错误。

python -m ace_teleop.dynamixel.calibration.test_calibration --port /dev/serial/by-id/usb-FTDI_USB**-**Serial_Converter_FT8J0QI3-if00-port0 --type left
测试校准后，进入 ace_teleop/configs/server 并更改相应 yml 文件中的 port。
查找摄像头索引并测试摄像头帧率
此代码将查找所有网络摄像头并显示帧率。
python

# 按“k”键显示下一个摄像头

python3 -m ace_teleop.tools.find_webcam
获取摄像头索引后，进入 ace_teleop/configs/server 并更改相应 yml 文件中的 cam_num。
详细信息
使用默认配置和自动映射，你可以轻松控制提供的机器人。以下详细信息可能有助于你充分利用 ACE 并更好地处理不同的任务。
初始化
双手配置： 以下姿势将开始初始化。如果操作员的双手没有保持静止或手指没有平展，初始化过程将重新开始。

初始化姿势
image
单臂配置： 按 “p” 键进行初始化。
映射控制
r： 将当前人体旋转重新映射到设定的机器人旋转。
p： 将当前人体位置重新映射到设定的机器人位置。
m： 第一次按下：机器人将保持静止。第二次按下：将当前人体姿势重新映射到当前机器人姿势。
x： 第一次按下：锁定 x 轴运动。第二次按下：解锁 x 轴运动。
y： 第一次按下：锁定 y 轴运动。第二次按下：解锁 y 轴运动。
z： 第一次按下：锁定 z 轴运动。第二次按下：解锁 z 轴运动。
配置
为了更好地控制机器人，进入 ace_teleop/configs/server 并修改相应 yml 文件中的 pos_scale、roll_scale、pitch_scale、yaw_scale。

## Acknowlegments

This code base refers a lot to many previous amazing works like [BunnyVisionPro](https://github.com/Dingry/bunny_teleop_server), [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision), [GELLO](https://github.com/wuphilipp/gello_software). Also, the codes are built on some superior public project, such as [pinocchio](https://github.com/stack-of-tasks/pinocchio) and [dex-retargeting](https://github.com/dexsuite/dex-retargeting).
