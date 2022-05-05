1.ros_arduino_bridge 概述
--------
该功能包包含Arduino库和用来控制Arduino的ROS驱动包，它旨在成为在ROS下运行Arduino控制的机器人的完整解决方案。

其中当前主要关注的是:功能包集中一个兼容不同驱动的机器人的基本控制器（base controller），它可以接收ROS Twist类型的消息，可以发布里程数据到ROS端。

**1.1 特点：**

* 直接支持 Ping 声纳和Sharp红外 (GP2D12) 传感器

* 也可以从通用的模拟和数字信号的传感器读取数据

* 可以控制数字输出（例如打开和关闭开关或 LED）

* 支持PWM伺服机

* 如果使用所要求的硬件的话，可以配置基本的控制

* 如果你的Arduino编程基础好的话，并且具有python基础的话，你就可以很自由的改动代码来满足你的硬件要求

* * 该库包括一个用于差速驱动机器人的Base Controller，该控制器接受 ROS Twist 消息并将 Odom 里程计数据发布回 PC。Base Controller需要使用电机控制器和编码器来读取里程数据。库的当前版本支持以下基本控制器硬件：


**1.2 注意：**

* Robogaia Mega Encoder shield 只能与 Arduino Mega 一起使用。

* 板上编码计数器（ARDUINO_ENC_COUNTER）目前仅支持Arduino Uno

* 上面非硬性规定，有一定的编程基础，你也可以按需更改

**1.3 模块：**

* ros_arduino_bridge：   metapackage (元功能包)，catkin_make安装即可

* ros_arduino_msgs：     #消息定义包

* ros_arduino_firmware： 固件包，更新到Arduino（执行运动指令、发送电机编码器数据，通过serial与上位机通讯）

* ros_arduino_python：   #ROS相关的Python包，用于上位机，树莓派等开发板或电脑等（监听/cmd_vel并转换为移动指令下发给电机；将电机编码器数据转换为里程计数据发到/odom）


**1.4 文件结构：**

├── ros_arduino_bridge                      # metapackage (元功能包)

    ├── CMakeLists.txt

    └──  package.xml

├── ros_arduino_firmware                    #固件包，更新到Arduino

    ├── CMakeLists.txt

    ├── package.xml

    └──  src

        └── libraries                       #库目录

            ├── MegaRobogaiaPololu          #针对Pololu电机控制器，MegaRobogaia编码器的头文件定义

                ├── commands.h              #定义命令头文件

                ├── diff_controller.h       #差分轮PID控制头文件

                ├── MegaRobogaiaPololu.ino  #PID实现文件

                ├── sensors.h               #传感器相关实现，超声波测距，Ping函数

                └──  servos.h                #伺服器头文件

            └── ROSArduinoBridge            #Arduino相关库定义

                ├── commands.h              #定义命令

                ├── diff_controller.h       #差分轮PID控制头文件

                ├── encoder_driver.h        #编码器驱动头文件

                ├── encoder_driver.ino      #编码器驱动实现, 读取编码器数据，重置编码器等

                ├── motor_driver.h          #电机驱动头文件

                ├── motor_driver.ino        #电机驱动实现，初始化控制器，设置速度

                ├── ROSArduinoBridge.ino    #核心功能实现，程序入口

                ├── sensors.h               #传感器头文件及实现

                ├── servos.h                #伺服器头文件，定义插脚，类

                └── servos.ino              #伺服器实现

├── ros_arduino_msgs                        #消息定义包

    ├── CMakeLists.txt

    ├── msg                                 #定义消息

        ├── AnalogFloat.msg                 #定义模拟IO浮点消息

        ├── Analog.msg                      #定义模拟IO数字消息

        ├── ArduinoConstants.msg            #定义常量消息

        ├── Digital.msg                     #定义数字IO消息

        └──  SensorState.msg                 #定义传感器状态消息

    ├── package.xml

    └──  srv                                 #定义服务

        ├── AnalogRead.srv                  #模拟IO输入

        ├── AnalogWrite.srv                 #模拟IO输出

        ├── DigitalRead.srv                 #数字IO输入

        ├── DigitalSetDirection.srv　　　　 #数字IO设置方向

        ├── DigitalWrite.srv                #数字IO输入

        ├── ServoRead.srv                   #伺服电机输入

        └── ServoWrite.srv                  #伺服电机输出

└── ros_arduino_python                      #ROS相关的Python包，用于上位机，树莓派等开发板或电脑等。

    ├── CMakeLists.txt

    ├── config                              #配置目录

        └──  arduino_params.yaml             #定义相关参数，端口，rate，PID，sensors等默认参数。由arduino.launch调用

    ├── launch

        └──  arduino.launch                  #启动文件

    ├── nodes

        └──  arduino_node.py                 #python文件,实际处理节点，由arduino.launch调用，即可单独调用。

    ├── package.xml

    ├── setup.py

    └── src                                 #Python类包目录

        └── ros_arduino_python

            ├── arduino_driver.py           #Arduino驱动类

            ├── arduino_sensors.py          #Arduino传感器类

            ├── base_controller.py          #基本控制类，订阅cmd_vel话题，发布odom话题

            └── __init__.py                 #类包默认空文件


上述目录结构虽然复杂，但是关注的只有两大部分:

* ros_arduino_bridge/ros_arduino_firmware/src/libraries/ROSArduinoBridge
* ros_arduino_bridge/ros_arduino_python/config/arduino_params.yaml

前者是Arduino端的固件包实现，需要修改并上传至Arduino电路板；

后者是ROS端的一个配置文件，相关驱动已经封装完毕，我们只需要修改配置信息即可。

整体而言，借助于 ros_arduino_bridge 可以大大提高我们的开发效率。


**1.5 官方 ROS 文档** 

可以在 ROS wiki 上找到此文档的标准 ROS 样式版本：

http://www.ros.org/wiki/ros_arduino_bridge



2.准备工作
-------------------
**2.1 Python Serial:** 

ros_arduino_bridge 依赖于 python-serial 功能包，请提前安装此包。

在Ubuntu下安装python-serial包，使用命令：

    $ sudo apt-get install python-serial

在非 Ubuntu 系统上，使用：

    $ sudo pip install --upgrade pyserial

or

    $ sudo easy_install -U pyserial


**2.2 Arduino IDE 1.6.6 或更高版本:**

请注意，条件 #include 语句的预处理在早期版本的 Arduino IDE 中被破坏。为确保 ROS Arduino Bridge 固件正确编译，请务必安装 1.6.6 或更高版本的 Arduino IDE。您可以从https://www.arduino.cc/en/Main/Software下载 IDE 。


**2.3 硬件:**

固件应与任何兼容 Arduino 的控制器配合使用，以读取传感器和控制 PWM 伺服系统。但是，要使用Base Controller，您将需要上述支持的电机控制器和编码器硬件。如果您没有这个硬件，您仍然可以尝试读取传感器和控制伺服系统的软件包。有关如何执行此操作的说明，请参阅本文档末尾的 NOTES 部分。

要使用Base Controller，您还必须为电机控制器和编码器安装适当的库。对于 Pololu VNH5019 双电机屏蔽，可以在以下位置找到库：

https://github.com/pololu/Dual-VNH5019-Motor-Shield

对于 Pololu MC33926 双电机屏蔽，可以在以下位置找到库：

https://github.com/pololu/dual-mc33926-motor-shield

Robogaia Mega Encoder 库可以在以下位置找到：

http://www.robogaia.com/uploads/6/8/0/9/6809982/__megaencodercounter-1.3.tar.gz

L298 电机驱动器不需要任何库

这些库应该安装在您的标准 Arduino 素描本/库目录中。

最后，假设您使用的是 1.0 或更高版本的 Arduino IDE。



**2.4 在 Linux 下准备你的串口:**

您的 Arduino 可能会作为端口 /dev/ttyACM# 或 /dev/ttyUSB# 连接到您的 Linux 计算机，其中 # 是一个数字，如 0、1、2 等，具体取决于连接的其他设备的数量。做出决定的最简单方法是拔下所有其他 USB 设备，插入您的 Arduino，然后运行命令：

    $ ls /dev/ttyACM*

or 

    $ ls /dev/ttyUSB*

希望这两个命令之一将返回您正在寻找的结果（例如 /dev/ttyACM0），另一个将返回错误“没有这样的文件或目录”。

接下来，您需要确保您对端口具有读/写访问权限。假设您的 Arduino 已连接到 /dev/ttyACM0，请运行以下命令：

    $ ls -l /dev/ttyACM0

您应该会看到类似于以下内容的输出：

    crw-rw---- 1 root work 166, 0 2013-02-24 08:31 /dev/ttyACM0

请注意，只有 root 和“work”组具有读/写访问权限。因此，您需要成为work组的成员。您只需执行一次此操作，然后它应该适用于您稍后插入的所有 USB 设备。

要将自己添加到work组，请运行以下命令：

    $ sudo usermod -a -G work your_user_name

其中 your_user_name 是您的 Linux 登录名。您可能必须退出 X-window 会话，然后重新登录，或者如果您想确定，只需重新启动计算机。

当您再次登录时，请尝试以下命令：

    $ groups

您应该会看到您所属的组列表，包括work。


**2.5 安装 ros_arduino_bridge 库:**

    $ cd ~/catkin_ws/src
    $ git clone git@github.com:yanjingang/ros_arduino_bridge.git
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ~/catkin_ws/devel/setup.bash


提供的 Arduino 库称为 ROSArduinoBridge，位于 ros_arduino_firmware/src/libraries/ROSArduinoBridge 中。此示例库有特殊的硬件要求，但也可以通过关闭BaseController（如本文档末尾的 NOTES 部分所述）与其他 Arduino 类型板（例如 Uno）一起使用。



**2.6.固件命令**

ROSArduinoLibrary 通过串行端口接受单字母命令，用于轮询传感器、控制伺服系统、驱动机器人和读取编码器。这些命令可以通过任何串行接口发送到 Arduino，包括 Arduino IDE 中的串行监视器。

**注意：** 在尝试这些命令之前，使用串行监视器窗口右下角的两个下拉菜单将串行监视器波特率设置为 57600，并将行终止符设置为“回车”或“NL & CR”。

命令列表可以在文件commands.h 中找到。目前的名单包括：

<pre>
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
</pre>

例如，要获取引脚 3 上的模拟读数，请使用以下命令：

a 3

要将数字引脚 3 的模式更改为 OUTPUT，请发送命令：

c 3 1

获取当前编码器计数：

e

要以每秒 20 个编码器标记/秒 向前移动机器人：

m 20 20


**2.7 测试您的接线连接**

在差动驱动机器人上，电机连接到极性相反的电机控制器端子。类似地，来自编码器的 A/B 引线以相反的方式相互连接。但是，您仍然需要确保 
(a) 在给定正电机速度时车轮向前移动，以及 (b) 当车轮向前移动时编码器计数增加。

之后将你的机器人放到地上，您可以使用 Arduino IDE 中的串行监视器来测试这两个要求。使用“m”命令激活电机，使用“e”命令获取编码器计数，使用“r”命令将编码器重置为 0。请记住，在固件级别，电机速度以每个编码器滴答数给出其次，对于编码器分​​辨率，例如每轮旋转 4000 次计数，诸如“m 20 20”之类的命令应该相当缓慢地移动轮子。（轮子只会移动 2 秒，这是 AUTO_STOP_INTERVAL 的默认设置。）还要记住，第一个参数是左电机速度，第二个参数是右电机速度。类似地，当使用“e”命令时，返回的第一个数字是左编码器计数，第二个数字是右编码器计数。

最后，您可以使用“r”和“e”命令通过手动旋转轮子大约一整圈并检查报告的计数来验证预期的编码器计数。



3.案例实现
-----------------------------------
基于ros_arduino_bridge的底盘实现具体步骤如下:

* 了解并修改Arduino端程序主入口ROSArduinoBridge.ino 文件；

* Arduino端添加编码器驱动；

* Arduino端添加电机驱动模块；

* Arduino端实现PID调试；

* ROS端修改配置文件。

**3.1 配置 ROSArduinoBridge 节点**

编辑ROSArduinoBridge.ino文件，打开底盘控制、L298电机驱动，关闭舵机
<pre>
//是否启用底盘控制器
#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
//启用基座控制器需要设置的电机驱动以及编码器驱动
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   /* L298P电机驱动版 Motor driver*/
   #define L298P_MOTOR_DRIVER
   
   /* L298N Motor driver*/
   //#define L298N_MOTOR_DRIVER

#endif

//是否启用舵机
//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos
</pre>


**3.2 配置 ros_arduino_python 节点**

现在您的 Arduino 正在运行所需的示例，您可以在您的 PC 上配置 ROS 端。您可以通过编辑目录 ros_arduino_python/config 中的 YAML 文件来定义机器人的尺寸、PID 参数和传感器配置。所以首先进入该目录：

    $ roscd ros_arduino_python/config

现在将提供的配置文件复制到一个您可以修改的文件中：

    $ cp arduino_params.yaml my_arduino_params.yaml

在您喜欢的文本编辑器中打开您的 params 文件副本 (my_arduino_params.yaml)。它应该开始看起来像这样：

<pre>
port: /dev/ttyUSB0
baud: 57600
timeout: 0.1

rate: 50
sensorstate_rate: 10

use_base_controller: False
base_controller_rate: 10

# === 机器人传动系统参数 Robot drivetrain parameters
#wheel_diameter: 0.146
#wheel_track: 0.2969
#encoder_resolution: 8384 # from Pololu for 131:1 motors
#gear_reduction: 1.0
#motors_reversed: True

# === PID parameters
#Kp: 20
#Kd: 12
#Ki: 0
#Ko: 50
#accel_limit: 1.0

# === 传感器定义。仅示例 - 请为您的机器人单独设置 Sensor definitions.  Examples only - edit for your robot.
#     传感器类型可以是以下之一（区分大小写！） Sensor type can be one of the follow (case sensitive!):
#	  * Ping
#	  * GP2D12
#	  * Analog
#	  * Digital
#	  * PololuMotorCurrent
#	  * PhidgetsVoltage
#	  * PhidgetsCurrent (20 Amp, DC)

sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  arduino_led:          {pin: 13, type: Digital, rate: 5, direction: output}
}
</pre>

**注意：** 不要在 .yaml 文件中使用选项卡，否则解析器会在尝试加载它时将其退回。请始终使用空格代替。 
另外：定义传感器参数时，列表中的最后一个传感器在行尾没有逗号 (,)，但其余所有传感器都必须有逗号。

现在让我们看看这个文件的每个部分。

_端口设置_

端口可能是 /dev/ttyACM0 或 /dev/ttyUSB0。相应地设置。

The MegaRobogaiaPololu Arudino sketch connects at 57600 baud by default.

_轮询速率_

主要速率参数（默认为 50 Hz）决定了外部 ROS 循环的运行速度。在大多数情况下，默认值就足够了。无论如何，它应该至少与您最快的传感器速率（定义如下）一样快。

该sensorstate_rate决定如何经常发布所有传感器读数的汇总清单。每个传感器还发布自己的主题和速率。

该use_base_controller参数默认设置为False。将其设置为 True 以使用base control（假设您有所需的硬件。）您还必须设置后面的 PID 参数。

该base_controller_rate决定如何经常发布里程计读数。

_定义传感器_

传感器参数是定义传感器名称和传感器参数的字典。（您可以随意命名每个传感器，但请记住，传感器的名称也将成为该传感器的主题名称。）

四个最重要的参数是引脚、类型、速率和方向。该速率定义了每秒要轮询该传感器的次数。例如，电压传感器可能每秒只轮询一次（甚至每 2 秒一次：rate=0.5），而声纳传感器可能每秒轮询 20 次。类型必须是列出的类型之一（区分大小写！）。默认方向是输入，以便定义输出引脚，将方向明确设置为输出。在上面的示例中，Arduino LED（引脚 13）将以每秒 2 次的速率打开和关闭。

_设置传动系统和 PID 参数_

要使用base control，您必须取消注释并设置机器人传动系统和 PID 参数。示例传动系统参数适用于相距 11.5" 的 6" 驱动轮。请注意，ROS 使用米表示距离，因此相应地进行转换。示例编码器分辨率（每转标记）来自 Pololu 131:1 电机的规格。为您的电机/编码器组合设置适当的编号。如果您发现车轮向后转动，请将 motor_reversed 设置为 True，否则设置为 False。

PID 参数设置起来比较棘手。您可以从示例值开始，但请确保在向其发送第一个 Twist 命令之前将您的机器人放在块上。


**3.3 启动 ros_arduino_python 节点**

查看 ros_arduino_python/launch 目录中的启动文件 arduino.launch。如您所见，它指向一个名为 my_arduino_params.yaml 的配置文件。如果您将配置文件命名为不同的名称，请更改启动文件中的名称。

连接 Arduino 并运行 MegaRobogaiaPololu 草图后，使用您的参数启动 ros_arduino_python 节点：

    $ roslaunch ros_arduino_python arduino.launch

您应该会看到类似于以下输出的内容：

<pre>
process[arduino-1]: started with pid [6098]
Connecting to Arduino on port /dev/ttyUSB0 ...
Connected at 57600
Arduino is ready.
[INFO] [WallTime: 1355498525.954491] Connected to Arduino on port /dev/ttyUSB0 at 57600 baud
[INFO] [WallTime: 1355498525.966825] motor_current_right {'rate': 5, 'type': 'PololuMotorCurrent', 'pin': 1}
[INFO]
etc
</pre>

如果你的机器人上有任何 Ping 声纳传感器并且你在你的配置文件中定义了它们，它们应该开始闪烁以表明你已经建立了连接。

**3.4 查看传感器数据**

要查看汇总的传感器数据，请回显传感器状态主题：

    $ rostopic echo /arduino/sensor_state

要查看任何特定传感器的数据，请回显其主题名称：

    $ rostopic echo /arduino/sensor/sensor_name

例如，如果您有一个名为 ir_front_center 的传感器，您可以使用以下命令查看其数据：

    $ rostopic echo /arduino/sensor/ir_front_center

您还可以使用 rxplot 绘制范围数据：

    $ rxplot -p 60 /arduino/sensor/ir_front_center/range


**3.5 发送 Twist 命令和查看 odom 里程计数据**

将您的机器人放在地上，然后尝试发布 Twist 命令：
    
    $ rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    
    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{ angular: {z: 1} }'

车轮应按与逆时针旋转一致的方向转动（右轮向前，左轮向后）。如果它们转向相反的方向，请将配置文件中的 motor_reversed 参数设置为其当前设置的相反方向，然后终止并重新启动 arduino.launch 文件。

使用以下命令停止机器人：

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'

查看里程计数据：

    $ rostopic echo /odom

or

    $ rxplot -p 60 /odom/pose/pose/position/x:y, /odom/twist/twist/linear/x, /odom/twist/twist/angular/z

**3.6 ROS服务 ROS Services**

ros_arduino_python 包还定义了一些 ROS 服务，如下所示：

**digital_set_direction** - 设置数字引脚的方向

    $ rosservice call /arduino/digital_set_direction pin direction

其中 pin 是引脚编号，方向为 0 表示输入，1 表示输出。
where pin is the pin number and direction is 0 for input and 1 for output.

**digital_write** - 向数字引脚发送低 (0) 或高 (1) 信号

    $ rosservice call /arduino/digital_write pin value

其中 pin 是引脚编号，值为 0 表示低电平，1 表示高电平。
where pin is the pin number and value is 0 for LOW and 1 for HIGH.

**伺服写入** - 设置伺服的位置

    $ rosservice call /arduino/servo_write id pos

其中 id 是 Arduino 草图 (servos.h) 中定义的伺服索引，pos 是弧度 (0 - 3.14) 中的位置。

**servo\_read** -读伺服的位置

    $ rosservice call /arduino/servo_read id

其中 id 是 Arduino 草图 (servos.h) 中定义的伺服索引


**3.7 使用板载车轮编码器计数器（仅限 Arduino Uno）**

该固件支持 Arduino Uno 的板载车轮编码器计数器。这允许将车轮编码器直接连接到 Arduino 板，而无需任何额外的车轮编码器计数器设备（例如 RoboGaia 编码器屏蔽）。

为了速度，代码直接寻址特定的 Atmega328p 端口和中断，使此实现依赖于 Atmega328p（Arduino Uno）。（不过，它应该很容易适应其他板/AVR 芯片。）

要使用板载车轮编码器计数器，请将您的车轮编码器连接到 Arduino Uno，如下所示：

    Left wheel encoder A output -- Arduino UNO pin 2
    Left wheel encoder B output -- Arduino UNO pin 3

    Right wheel encoder A output -- Arduino UNO pin A4
    Right wheel encoder B output -- Arduino UNO pin A5

在 ROSArduinoBridge 草图中进行以下更改以禁用 RoboGaia 编码器屏蔽，并启用板载编码器屏蔽：

    /* The RoboGaia encoder shield */
    //#define ROBOGAIA
    /* Encoders directly attached to Arduino board */
    #define ARDUINO_ENC_COUNTER

编译更改并上传到您的控制器。

**3.8 使用 L298 电机驱动器 Using L298 Motor driver**

L298电机驱动器和arduino板之间的接线在固件中的motor_driver.h中定义如下：

    #define RIGHT_MOTOR_BACKWARD 5
    #define LEFT_MOTOR_BACKWARD  6
    #define RIGHT_MOTOR_FORWARD  9
    #define LEFT_MOTOR_FORWARD   10
    #define RIGHT_MOTOR_ENABLE 12
    #define LEFT_MOTOR_ENABLE 13

以这种方式连接它们或根据需要更改它们，并确保定义了 L298 电机驱动程序，然后编译并上传固件。


9.笔记 NOTES
-----
如果您没有运行base control器所需的硬件，请按照以下说明操作，以便您仍然可以使用与 Arduino 兼容的控制器读取传感器和控制 PWM 伺服系统。

首先，您需要编辑 ROSArduinoBridge 示例。在文件的顶部注释掉这一行：

<pre>
#define USE_BASE
</pre>

所以它看起来像这样：

<pre>
//#define USE_BASE
</pre>

**注意：** 如果您使用的是 1.6.6 之前的 Arduino IDE 版本，您还需要注释掉文件 encoder_driver.ino 中如下所示的行：

    #include "MegaEncoderCounter.h"

所以它看起来像这样：

    //#include "MegaEncoderCounter.h"

编译更改并上传到您的控制器。

接下来，编辑 my_arduino_params.yaml 文件并确保 use_base_controller 参数设置为 False。这里的所有都是它的。
