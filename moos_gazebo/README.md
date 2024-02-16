# moos_gazebo
Package to launch CORA, WAMV, or Heron USVs in custom configurations. Examples launch with a datum point associated with the MIT Sailing Pavilion

## Launch Files
Launch files for each vehicle as well as sample launch files for full simulation

### cora.launch
Base launch script for the custom CORA vessel
#### ROS Args:
  * `urdf` Argument to define where to look for vehicle URDF
  * `root_namespace` Argument to prefix root_namespace  (ex: asset_1), used for namespacing in ROS
  * `namespace` Argument to prefix namespace, used in creation of multiple assets  (ex: cora)
  * `tf_prefix` Argument to prefix name of vessel, Ex: asset_1
  * `sensors` Argument to turn off perception sensors on the vessel
  * `gateway_key` Argument to prefix _NODE_REPORT_GAZEBO msg sent by `compose_contact_report` to MOOS-IvP
  * `right_thrust_key` Argument to process right thrust data from MOOS-IvP
  * `left_thrust_key` Argument to process left thrust data from MOOS-IvP
  * `gateway_port` Argument to set the port for `protobuf_client` to communicate with the MOOS-IvP vesse
l community
  * `USV location and attitude` Arguments to set vessel location `x,y,z,R,P,Y`

### wamv.launch
Base launch script for the custom WAM-V vessel
#### ROS Args:
  * `urdf` Argument to define where to look for vehicle URDF
  * `root_namespace` Argument to prefix root_namespace  (ex: asset_1), used for namespacing in ROS
  * `namespace` Argument to prefix namespace, used in creation of multiple assets  (ex: wamv)
  * `tf_prefix` Argument to prefix name of vessel, Ex: asset_2
  * `sensors` Argument to turn off perception sensors on the vessel
  * `gateway_key` Argument to prefix _NODE_REPORT_GAZEBO msg sent by `compose_contact_report` to MOOS-IvP
  * `right_thrust_key` Argument to process right thrust data from MOOS-IvP
  * `left_thrust_key` Argument to process left thrust data from MOOS-IvP
  * `gateway_port` Argument to set the port for `protobuf_client` to communicate with the MOOS-IvP vessel community
  * `USV location and attitude` Arguments to set vessel location `x,y,z,R,P,Y`


### mit-SP_hello-world.launch and mit-SP_multi-robot.launch
 * Top level launch script to launch a single USV model for use with MOOS-IvP. To be used in conjunction with the `moos-ivp-sim-gazebo/missions/hello-world` mission
 * Launch Params
   * `vessel_1_type` Argument to set vessel type to launch associated vessel launch script
   * `vessel_1_urdf` Argument to set vessel model
 * Launch Example
   * $ roslaunch moos_gazebo mit-SP_hello-world.launch vessel_1_type:=wamv vessel_1_urdf:=/home/<USER>/<ROS_WS>/src/gazebo_sim_moos_ivp/moos_gazebo/config/wamv/wamv.xacro
   * $ roslaunch moos_gazebo mit-SP_hello-world.launch vessel_1_type:=cora vessel_1_urdf:=/home/<USER>/<ROS_WS>/src/gazebo_sim_moos_ivp/moos_gazebo/config/cora/cora.xacro
 * Copy launch script to your own ROS package and edit for a custom mission

#### ROS Args:
  In addition to the base level args in cora.launch
  * `world` Argument to launch a Gazebo world model
    See `../worlds/mit_SP.world` as an example

## Vehicle and Sensor URDF
`config:` Vehicle and sensor URDF files

## cora
CORA vehicle URDF files
  * Note these files are modified to be used within these packages

### cora.xacro
File is used to call the base CORA vehicle URDF and sensor MACROS. Copy and edit file for custom vehicle setup

## wamv
WAM-V vehicle URDF files
  * Note these files are modified to be used within these packages

### wamv.xacro
File is used to call the base WAM-V vehicle URDF and sensor MACROS. Copy and edit file for custom vehicle setup
