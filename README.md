# Branch Detection Worksapce

This is a ROS2 workspace. Various packages have been provided for the development of ToF branch detection.

In order to run this package, a microROS package must be created. Some steps from the microROS tutorials can be omitted.

```
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Enter the branch_detection_ws workspace and download the micro-ROS tools
cd branch_detection_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Next, create the microROS agent:

```
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

Finally, build the agent and source the package.

```
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### Running the microROS agent

The agent is launched from the `teensy32_tof_bringup` package, but can be individually run using the following command:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 ROS_DOMAIN_ID=0
```