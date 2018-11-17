# HIOB example client

## Description

Provides an example client for the [HIOB-ROS-Server](https://github.com/theCalcaholic/hiob_ros), which tracks objects
within some camera footage (e.g. from a webcam).

__Note:__ _This is only a client application; you need to have an instance of the
[server](https://github.com/theCalcaholic/hiob_ros) running somewhere in the network (or on the same machine)
in order to use it._

## Requirements
- Python 2
- ROS (Kinetic or above)
- hiob_msgs (see [**Installation**](#installation) section)


---------------------------------
__Note:__ _The following instructions assume a modern Linux OS. However, while not tested explicitly,
the client might likely be compatible with other operating systems as well, if they have Python and ROS available._
---------------------------------

# Installation

1. Source the ROS setup script of the ROS distro, e.g.
    ```sh
    . /opt/ros/kinetic/setup.bash
    ```
2. Create or reuse a directory for a ROS workspace, e.g.:
    ```sh
    mkdir -p my_ros_ws/src
    ```
3. Clone this repository and dependencies into an 'src' directory inside the created workspace:
    ```sh
    git clone https://github.com/theCalcaholic/hiob_example_client.git src/hiob_example_client
    git clone https://github.com/theCalcaholic/hiob_msgs.git src/hiob_msgs
    ```
4. Install dependencies of packages in the created workspace, e.g. (Replace 'kinetic' by your ros distro):
    ```sh
    rosdep install --from-paths src --ignore-src --rosdistro kinetic
    ```
5. Install the ROS packages with catkin (from your ros workspace root directory):
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