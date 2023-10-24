# moos_gazebo_nodes
Package used to publish simulated vessel data to MOOS-IvP

## ROS API: compose_contact_report
Node to parse contact vessel topics into a node report for MOOS consumption

### Subs:
NOTE: topics to be subscribed to will be prefixed with the `/root_namespace/namespace`
* `.../sensors/gps/gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
Positional data from GPS

* `.../sensors/gps/gps/fix_velocity` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3Stamped.html)
Velocity data from GPS

* `.../sensors/imu/imu/data` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))
IMU data

### Pubs:
* `/send_to_gateway` ([protobuf_client/msgs/Gateway])
Topic to be sent to MOOS-IvP. Minimal vessel data in the for of a MOOS Node Report in the form of a string `NODE_REPORT="NAME=name,LAT=lat,LON=lon,HDG=heading,SPD=speed,YAW=yaw,PITCH=pitch,ROLL=roll,DEP=depth"

### ROS Parameters:
* `namespace` Parameter to set namespace
* `root_namespace` Parameter to set root namespace
* `contact_length` Parameter to set vessel length
* 'gateway_key` Parameter to set gateway key for MOOS consumption
* `contact_gt` Parameter to enable groundtruth publish for MOOS consumption
TODO: Remove this as it is no longer used

## ROS API: publish_robot_data
Node to publish robot data rpy, heading, speed from various sources

### Subs:
NOTE: topics to be subscribed to will be prefixed with the `/root_namespace/namespace`

* `.../sensors/imu/imu/data` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))

* `.../sensors/gps/gps/fix_velocity` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3Stamped.html)

* `.../robot_localization/odometry/filtered` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
Position and Velocity output fused from the GPS/IMU 

### Pubs:
NOTE: topics to be published to will be prefixed with the `/root_namespace/namespace`

* `.../sensors/imu/imu/rpy`
Roll, Pitch, and Yaw data transformed from raw IMU

* `.../sensors/imu/imu/heading`
Heading data transformed from raw IMU

* `.../sensors/gps/gps/speed`
Speed data transformed from raw GPS velocity data

### ROS Parameters

* `root_namespace` Parameter to set root_namespace
* `namespace` Parameter to set namespace

## ROS API: thrust_vessel
Node to parse data coming in from protobuf_client into topics for ROS consumption

### Subs:

* `/gateway_msg` ([protobuf_client/msg/Gateway])
Message send by MOOS-IvP to be parsed for commanded thrust left/right

### Pubs:

* `/left_cmd` ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
* `/right_cmd` ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

### ROS Parameters

* `left_thrust_key` Key to parse from gateway for left thrust
* `right_thrust_key` Key to parse from gatway for right thrust
