# Brunhilde workspace
This is a ROS2 Jazzy Jalisco workspace for Brunhilde, the HCR Lab's quadruped based on the ODRI Solo8 robot.

The following packages are included:

- [brunhilde_description](/src/brunhilde_description): Contains xacro and configuration files for rviz and ODRI packages
- [brunhilde_bringup](/src/brunhilde_bringup): Contains launch files for starting all necessary services
- [brunhilde_control](/src/brunhilde_control): Integration of ros2 control. Contains necessary config and launch files and a [script to test the controllers](/src/brunhilde_control/brunhilde_control/testMovements.py)
- [brunhilde_gz_sim](/src/brunhilde_gz_sim): All files and tools needed to simulate the robot in Gazebo Sim Harmonic. Also includes configs for the gazebo ros bridge
- [brunhilde_teleop](/src/brunhilde_teleop): Contains configs for teleoperation using keyboard or joy twist
- [brunhilde_walk](/src/brunhilde_walk): Implements an open loop state machine for executing motions while being controlled by the user

The ODRI dependecies are included as submodules in the [solo8_packages](/src/solo8_packages) directory.

## Docker
To run the workspace in a docker container, you can use the provided [Dockerfile](/docker/Dockerfile). It runs a Ubuntu 24.04 image and installs all dependencies automatically. After building the image and running the container, navigate into the workspace folder, build the workspace with colcon and source the setup.bash file.

## Dependencies
The original project repositories use the package manager [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep). To make it easier to run our workspace inside a docker container we built the project without treep.

### ROS Packages
The workspace depends on the following ROS packages:

```ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher ros-jazzy-xacro gazebo ros-jazzy-gazebo-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-imu-tools```

They can be installed through Ubuntus package manager after adding the ROS2 repositories.

### ODRI Packages
The following packages are needed:
- [odri_control_interface](https://github.com/open-dynamic-robot-initiative/odri_control_interface)
- [master_board_sdk](https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk)
The master_board_sdk is a subfolder of the [master board repository](https://github.com/open-dynamic-robot-initiative/master-board)
- [ros2_hardware_interface_odri](https://github.com/Nordegraf/ros2_hardware_interface_odri) We forked the original repository to make it compatible with ROS2 Jazzy.

The following packages are needed as dependencies and are availabe through the ubuntu repositories:
- [boost](https://www.boost.org/)
- [Doxygen](https://www.doxygen.nl/index.html)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [eigenpy](https://github.com/stack-of-tasks/eigenpy)
- [Pybind11](https://github.com/pybind/pybind11)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

To install them all at once just run:

```sudo apt install libboost-all-dev doxygen libeigen3-dev ros-jazzy-eigenpy pybind11-dev libyaml-cpp-dev```

## Installation
with all dependencies installed, clone the repository and build the workspace with colcon:

```colcon build```

Don't forget to setup the workspace after building. From inside the workspace folder you need to run

```source install/setup.bash```

## Testing the workspace
To test if the workspace is set up correctly, you can run a gazebo simulation and some test movements. First launch the gazebo simulation with:

```ros2 launch brunhilde_gz_sim gz_sim.launch.py```

Then launch the controllers:

```ros2 launch brunhilde_control sim_control.launch.py```

And then execute the test movements script:

```ros2 run brunhilde_control testMovements.py```

You should see the robot laying down and then standing up again.

