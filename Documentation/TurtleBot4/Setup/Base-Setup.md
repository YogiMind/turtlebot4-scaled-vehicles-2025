# Base Setup
Setting up the Turtlebot and ROS2.
## Overview
1. [ROS2](#ros2)
    1. [PC](#pc)
    2. [Command Explanation](#command-explanation)
2. [Turtlebot](#turtlebot)
    1. [PC](#pc-1)
    2. [Robot](#robot)
3. [Read More](#read-more)

## ROS2
ROS2 is the Operating System on which the Turtlebot runs. This should be installed before you start working with Turtlebot 4 and includes important software such as [RViz](/Documentation/TurtleBot4/Software/RViz/RViz.md). 

### PC
The first step is to make sure that your system is up to date. Install any pending system updates before installing ROS2. If you want to understand the commands being ran, go to the [command explanation](#command-explanation) section. 

Once you've updated your computer's operating system, run the command `sudo apt install software-properties-common` which is used for handling sources and repositories from different software vendors more easily. Now run `sudo add-apt-repository universe`. 

Run `sudo apt update && sudo apt install curl -y sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg` to add the ROS2 [GPG Key](https://www.gnupg.org/gph/en/manual/c14.html). 

To add the repo to our list of sources, we run `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null` 

Run `sudo apt update` and `sudo apt upgrade`, then run `sudo apt install ros-humble-desktop` to install necessary software such as [RViz](/Documentation/TurtleBot4/Software/RViz/RViz.md), ROS etc.

Now, make sure to source your setup with `source /opt/ros/humble/setup.bash` and you are ready to start setting up the [Turtlebot](#turtlebot)

#### Command Explanation

`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"` This command tells the system where to find the ROS packages. The `dpkg --print-architecture` and `$(. /etc/os-release && echo $UBUNTU_CODENAME)` command is used to get the system's architecture and codename of the installed Ubuntu version. 

`| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null` This ptakes the output from the echo command and writes it to the `/etc/apt/sources.list.d/ros2.list` file. The `tee` command is used to write to files with `sudo` privileges. The `> /dev/null` part discards the standard output of `tee`.

## Turtlebot
**Make sure you have set up ROS2 and that it's running properly before you proceed with these steps.** 

### PC
Run `sudo apt update` followed by `sudo apt install ros-humble-turtlebot4-desktop`. This will install the `turtlebot4_desktop` package. 

### Robot
Follow the steps in [Network Discovery Server](/Documentation/TurtleBot4/Setup/Network-Discovery-Server.md). 

## Read More
[Turtlebot Basic Setup](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)  
[ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)  
[ROS2 Humble Installation Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)  
[Ubuntu about Repositories](https://help.ubuntu.com/community/Repositories/Ubuntu)
[GPG Keys](https://www.gnupg.org/gph/en/manual/c14.html)