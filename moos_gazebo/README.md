# moos_gazebo
Package to launch CORA and WAMV vessels in custom configurations. Examples launch with a datum point associated with the MIT Sailing Pavilion


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
