# brunhilde_gz_sim
This package contains all files and tools needed to simulate the robot in Gazebo Sim Harmonic including a simulated IMU and bridging to ROS2.

## Launch Files
- gz_sim.launch.py: Launches the gazebo simulation with the robot model. The controllers must be started seperately. There is a [launch file](src/brunhilde_control/launch/sim_control.launch.py)
for that in the brunhilde_control package.
- spawn.launch.py: Spawns a robot in a running gazebo instance.

## Other contents
There is a [config file](src/brunhilde_gz_sim/config/gz_bridge.yaml) for the gazebo ros bridge and the [gazebo world](src/brunhilde_gz_sim/worlds/flat.sdf) file for simulation.