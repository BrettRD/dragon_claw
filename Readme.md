* Claw

A robotic claw.

The claw is modelled in openscad, parameters are exported to json, meshes to STL.
xacro loads the meshes and config into a robot model.
KDL provides pose planning

KDL reference:
https://github.com/tuuzdu/crab_project/blob/master/crab_body_kinematics/src/rpy_body_kinematics.cpp
crab_project needs to be flattend, migrated to ROS2, and converted to composable nodes.


