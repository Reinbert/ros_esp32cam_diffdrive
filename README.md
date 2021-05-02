# ros-esp32cam-diffdrive

This is a ROS 2 & Micro-ROS project to control an RC tank with differential drive (skid steering) with an ESP32-CAM module. 
I'm using an XBox 360 gamepad connected to a PC running *Ubuntu 20.04* and [ROS 2 Foxy](https://docs.ros.org/en/foxy/index.html) as tele operation input. 
This data is then sent via wifi to an ESP32 microcontroller running [Micro-ROS](https://micro.ros.org/) which transforms the input to PWM signals controlling the 2 motors.
At a later stage, the video of the ESP32-CAM module should be sent back to ROS 2 for further processing or display.

Currently, I'm using an *ESP32-DevKitC WROOM-32D* board which doesn't have a built-in camera, but is slightly easier to handle than an ESP32-CAM module because of built-in buttons and a Micro-USB socket. 

**Note: This is work in progress and doesn't include all features yet.**

## ROS 2 & micro-ROS Installation

The first few steps are taken directly from the [micro-ROS Tutorials](https://micro.ros.org/docs/tutorials/core/first_application_linux/). 
(If the instructions depicted here don't work anymore, view the original tutorials for updates and maybe drop me a note.)

Install **ROS 2 Foxy FitzRoy** on your Ubuntu 20.04 LTS computer. I recommend installation via Debian packages, as instructed 
[here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).

Once you have a ROS 2 installation in the computer, follow these steps to install the micro-ROS build system:

```bash
# Source the ROS 2 installation, if you haven't added it to your .bashrc file.
source /opt/ros/foxy/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

## Creating the micro-ROS Agent

The [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) is responsible for the communication between ROS 2 on your PC and micro-ROS on your microcontroller. 
(It's similar to [rosserial_server](http://wiki.ros.org/rosserial_server) for ROS1.)
Run the following lines in your terminal:

```bash
# Download micro-ROS Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build agent and source the workspace again
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Creating the firmware workspace

The next steps are taken in part from this [blog post](https://discourse.ros.org/t/micro-ros-porting-to-esp32/16101), announcing the support of ESP32 devices by micro-ROS.

```bash
# Create a micro-ROS firmware supporting FreeRTOS and ESP32
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# Git clone this repository into 'microros_ws/firmware/freertos_apps/apps' directory   
git clone https://github.com/Reinbert/ros_esp32cam_diffdrive firmware/freertos_apps/apps/ros_esp32cam_diffdrive

# Configure the firmware by setting the ip address of the PC you run the micro-ROS Agent on. 
# (In most cases, this is the PC you are working on right now. Run 'ifconfig' in terminal to find out your ip address.) 
ros2 run micro_ros_setup configure_firmware.sh ros_esp32cam_diffdrive -t udp -i [agent ip address] -p 8888
# Configure more settings
ros2 run micro_ros_setup build_firmware.sh menuconfig
# Navigate to 'micro-ROS Transport Settings' / 'WiFi Configuration' menu and enter your WiFi SSID and password. Save your changes and exit the interactive menu.

# Build the firmware
ros2 run micro_ros_setup build_firmware.sh

# Connect your ESP32 to the computer and flash the firmware
ros2 run micro_ros_setup flash_firmware.sh
# If you encounter a connect loop like this: 
# 'Connecting........_____.....', you need to manually restart the microcontroller into boot mode. 
# To do this, connect GPIO 0 to ground while powering on / resetting the module.
# Other ESP32 boards include designated buttons, labeled 'BOOT': GPIO 0 -> ground and 'EN': reset. 
# (https://github.com/espressif/esptool/wiki/ESP32-Boot-Mode-Selection)
```

## Testing the ROS2 connection

Start the ros 2 agent with this command:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Then power on or reset the ESP device. 2 new lines should appear in the terminal informing you of a new client and a new communication session. 
If not, first find out if the board appears in your router's list of wifi devices. Typically, they announce themselves as 'espressif'. 
If you can't see it there either, make sure you set your wifi credentials correctly while configuring the firmware. 

Open a new terminal and run:
```bash
# Source the ROS 2 installation, if you haven't added it to your .bashrc file.
source /opt/ros/foxy/setup.bash

# List all available topics
ros2 topic list
```
It returns a list of all available topics in the current ROS 2 ecosystem and should include `/cmd_vel`.
To get detailed information about this topic run:

```bash
ros2 topic info /cmd_vel -v
```

## Controlling the robot

At this point, the robot is ready to be controlled with a teleop node. Open a new terminal and run:
```bash
# Source the ROS 2 installation, if you haven't added it to your .bashrc file.
source /opt/ros/foxy/setup.bash

# Launch a keyboard tele operation node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This node allows you to control the robot via keyboard input. The keys are listed upon startup of the node. 
However, this way of robot control isn't very efficient, because you can only go full speed or stop.
If you have a gamepad, you can use it to control the robot more elegantly:
(Be sure to press the *enable_button* on your gamepad)

```bash
# Launch a gamepad tele operation node with your gamepad configuration. 
# Configs are listed here: https://github.com/ros2/teleop_twist_joy/tree/foxy/config 
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Since I have an XBox 360 gamepad at home and I'm not fond of pressing an *enable_button*, I wrote my own launch file which is included in this repository as well. 
You can launch it by `cd`-ing into the directory and launching it directly, without the hassle of creating a package first.
(Drop me a note if you have a different gamepad and want to add its configuration here.)

```bash
# Change into teleop directory
cd microros_ws/firmware/freertos_apps/apps/ros_esp32cam_diffdrive/teleop/launch

# Launch teleop with special configuration
ros2 launch teleop-launch.py 
```

## Further plans

Originally, I wanted to use [PlatformIO](https://platformio.org/) to program the microcontroller. 
However, due to the build steps described in this [blog post](https://discourse.ros.org/t/micro-ros-porting-to-esp32/16101), I've abandoned PlatformIO for now. 
This also means, that I don't have a fully functioning IDE with code completion and insights yet and programmed this example more or less in a bland editor. 
You can definitely see that in the code ;)
So I'm trying to figure out how get all those IDE features back while still being able to build a micro-ROS app. 
 
Another step will be the use of a real ESP32-CAM module as stated in the name of the repo and making use of the camera. 
I guess, there are some extra firmware configurations needed to enable the camera and the PSRam of the board. 

Additionally, I'm not satisfied with the calculation of the left and right motor values, as the current configuration only outputs 50% of max speed when going forward. 
Some smarter mathematics with focus on fun instead of precise movement are needed, since the robot doesn't measure the wheel rotations or odometry anyway. Current calculation:

```C
float left = (linear - angular) / 2.0f;
float right = (linear + angular) / 2.0f;
```