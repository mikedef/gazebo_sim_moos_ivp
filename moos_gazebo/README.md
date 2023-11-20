# moos_gazebo
Package to launch CORA, WAMV, or Heron USVs in custom configurations. Examples launch with a datum point associated with the MIT Sailing Pavilion

## cora.launch
Base launch script for the custom CORA vessel
### ROS Args:                                                                                          
  * `root_namespace` Argument to prefix root_namespace                                                  
  * `namespace` Argument to prefix namespace, used in creation of multiple assets                       
  * `tf_prefix` Argument to prefix name of vessel, Ex: asset_1                                          
  * `sensors` Argument to turn off perception sensors on the vessel                                     
  * `gateway_key` Argument to prefix _NODE_REPORT_GAZEBO msg sent by `compose_contact_report` to MOOS-Iv
P                                                                                                       
  * `right_thrust_key` Argument to process right thrust data from MOOS-IvP                              
  * `left_thrust_key` Argument to process left thrust data from MOOS-IvP                                
  * `gateway_port` Argument to set the port for `protobuf_client` to communicate with the MOOS-IvP vesse
l community                                                                                             
  * `USV location and attitude` Arguments to set vessel location `x,y,z,R,P,Y`


## mit-SP_hello-world.launch and mit-SP_multi-robot.launch
 * Top level launch script to launch a single USV model for use with MOOS-IvP. To be used in conjunction with the `moos-ivp-sim-gazebo/missions/hello-world` mission
 * Launch Params
   * `vessel_1_type` Argument to set vessel type to launch associated vessel launch script
   * `vessel_1_urdf` Argument to set vessel model
 * Launch Example
   * $ roslaunch moos_gazebo mit-SP_hello-world.launch vessel_1_type:=wamv vessel_1_urdf:=/home/<USER>/<ROS_WS>/src/gazebo_sim_moos_ivp/moos_gazebo/config/wamv/wamv.urdf
   * $ roslaunch moos_gazebo mit-SP_hello-world.launch vessel_1_type:=cora vessel_1_urdf:=/home/<USER>/<ROS_WS>/src/gazebo_sim_moos_ivp/moos_gazebo/config/cora/cora.xacro
 * Copy launch script to your own ROS package and edit for a custom mission

### ROS Args:
  In addition to the base level args in cora.launch
  * `world` Argument to launch a Gazebo world model
    See `../worlds/mit_SP.world` as an example


## Create a custom WAMV URDF
  * roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$PWD/config/wamv/thruster_config.yaml component_yaml:=$PWD/config/wamv/component_config.yaml wamv_target:=<path-to-target-dir>/<name>.urdf

