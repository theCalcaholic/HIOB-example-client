#HIOB example client

## Description

Provides an example client for the [HIOB-ROS-Server](https://github.com/theCalcaholic/hiob_ros), which tracks objects
within some camera footage (e.g. from a webcam).

__Note:__ _This is only a client application; you need to have an instance of the
[server](https://github.com/theCalcaholic/hiob_ros) running somewhere in the network (or on the same machine)
in order to use it._

## Requirements
- ROS (Kinetic or above)
- hiob_msgs (see [**Installation**](#installation) section)


# Installation

1. Source the ROS setup script of the ROS distro, e.g.
    ```sh
    . /opt/ros/kinetic/setup.bash
    ```
2. Create or reuse a directory for a ROS workspace and change to it, e.g.:
    ```sh
    mkdir my_ros_ws
    cd my_ros_ws
    ```
3. Clone this repository and dependencies into the created workspace:
    ```sh
    clone https://github.com/theCalcaholic/hiob_example_client.git
    clone https://github.com/theCalcaholic/hiob_msgs.git
    ```
4. Install the ROS packages with catkin:
    ```sh
    catkin_make install
    ```

## Usage

_This section assumes that you have followed the steps in the [**Installation**](#installation) section carefully._

If so, you should be able these steps in order to run HIOB:

1. Source the ROS setup script of the ROS distro e.g.:
    ```sh
    . /opt/ros/kinetic/setup.bash
    ```
2. Start an instance of roscore
3. Open a new terminal window and source the ROS setup scripts of both the ROS distro and your ROS workspace, e.g.:
    ```sh
    . /opt/ros/kinetic/setup.bash
    . my_ros_workspace/install/setup.bash
    ```
4. Run the example client:
    ```sh
    rosrun hiob_example_client launch.py
    ```