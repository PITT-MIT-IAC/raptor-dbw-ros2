# raptor-dbw-ros2

This is the MIT-Pitt-RW team's fork of the Indy Autonomous Challenge, Base Software Group, version of the Raptor Driver by Wire ROS2 driver. The original is available [here.](https://gitlab.com/IACBaseSoftware/raptor-dbw-ros2)


The original fork is a continuation of the [raptor_dbw_ros repo](https://github.com/NewEagleRaptor/raptor-dbw-ros) but in ROS2. This is the product of transferring the ROS1 codebase to ROS2.

## Running `raptor_dbw_can`
The driver relies on receiving CAN msgs via [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan). See [`raptor.launch.py`](raptor_dbw_can/launch/raptor.launch.py) for an example launch file. Please note you will need to specify the CAN bus to connect to. By default, it is `can0`
