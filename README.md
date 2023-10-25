# gazebo_sim_moos_ivp ROS 1 Package
Repository to simulate a marine vessel in the Gazebo simulation environment. This repo is intended to be used with protobuf_client package as well as the MOOS-IvP moos-ivp-gateway and moos-ivp-sim-gazebo packages

* Available vessels
  * CORA
  * WAM-V (TODO: Add model)

## moos_gazebo
## Launch Files

### cora.launch
  Base launch script for a custom CORA model in Gazebo
### mit-SP_hello-world.launch
  Top level launch script to launch a single CORA model for use with MOOS-IvP. To be used in conjunction with the `moos-ivp-sim-gazebo/missions/hello-world` mission
### mit-SP_multi-robot.launch
  Top level launch script to launch multiple CORA models for use with MOOS-IvP. To be used in conjunction with the `moos-ivp-sim-gazebo/missions/multi-robot` mission

## World Files
### mit_SP.world
  * Water world with the MIT Sailing Pavilion datum set as `42.358436, -71.087448`
    * Sets wind and wave forcing 

## moos_gazebo_nodes
  * compose_contact_report.py
    * Node to send vessel navigation data to MOOS
  * publish_robot_data.py
    * Node to publish robot data as topics
  * desired_to_thrust.py (depreciated)
    * Node to publish MOOS-IvP thrust data to vessel left/right thrusters
  * thrust_vessel.py
    * Node to publish COMMANDED_THRUST_LEFT and COMMANDED_THRUST_RIGHT data directly from MOOS-IvP

## Notes
* 

## TODO:
* 
