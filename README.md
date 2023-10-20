# gazebo_sim_moos_ivp ROS 1 Package
Repository to simulate a marine vessel in the Gazebo simulation environment. This repo is intended to be used with protobuf_client package as well as the MOOS-IvP moos-ivp-gateway and moos-ivp-sim-gazebo packages

* Available vessels
  * WAM-V
  * CORA

## moos_gazebo

## moos_gazebo_nodes
  * compose_contact_report.py
    * Node to send vessel navigation data to MOOS
  * publish_robot_data.py
    * Node to publish robot data as topics
  * desired_to_thrust.py
    * Node to publish MOOS-IvP thrust data to vessel left/right thrusters

## Notes
* 

## TODO:
* 
