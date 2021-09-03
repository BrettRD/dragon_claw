* Claw

A robotic claw.

The claw is modelled in openscad, parameters are exported to json, meshes to STL, that all happens in https://github.com/BrettRD/kinematics_scad

This package contains kinematics and joint state launch files to drive the model from a joystick with a simple gait.

`ros2 launch claw claw.launch.py`

You may need to convert the stl files from ascii format to binary format for rviz2
