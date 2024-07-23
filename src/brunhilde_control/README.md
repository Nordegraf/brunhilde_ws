# brunhilde_control

This package contains everything needed for controlling the Brunhilde robot using ROS2 Control.

Currently only position control using the JointTrajectoryController is implemented.

## Launch Files
- control.launch.py: Launches the controller manager and the controllers for controlling the actual robot.
- sim_control.launch.py: Launches controllers needed for simulation in Gazebo.
- mock_control.launch.py: Launches controllers using the ros2 control mock hardware interface.

## Hardware Interface
We use the [ros2_hardware_interface](https://github.com/stack-of-tasks/ros2_hardware_interface_odri) as ros2 control hardware interface. As it's written for ROS2 Foxy we migrated it to ROS2 Jazzy in a [fork](https://github.com/Nordegraf/ros2_hardware_interface_odri).

## Reference Implementation
The script [testMovements.py](/src/brunhilde_control/brunhilde_control/testMovements.py) is a reference implementation for controlling the robot using the JointTrajectoryController. Implemented are simple standup and laydown movements, a jump, some simple leg test sequence and a wave motion based on sine waves.

To run it within gazebo first launch the gazebo simulation with:
```ros2 launch brunhilde_gz_sim gz_sim.launch.py```

Then launch the controllers:
```ros2 launch brunhilde_control sim_control.launch.py```

An then run the script:
```ros2 run brunhilde_control testMovements.py```