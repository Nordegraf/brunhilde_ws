# brunhilde_description
This package contains xacro and configuration files for rviz and ODRI packages.

If an urdf model is needed, it can be generated from the xacro files using the following command:
```xacro brunhilde.urdf.xacro > brunhilde.urdf```

There is also a [config file](config/config_solo8.yaml) for the ODRI robot properties package.
for using ODRIs ros2 control hardware interface with the robot.

## Xacro Arguments
The xacro files use the following arguments:
- 'use_sim_hardware': if true the gazebo ros2 control interface is used.
- 'use_mock_hardware': if true the ros2 control mock hardware interface is used.
- 'use_real_hardware': default true. If true, the odri hardware interface is used.

Make sure to only set one of this arguments to true. Otherwise some services won't start or might fail.

# Copyright Notice
The xacro files are based on the ones provided by ODRI in the [robot_properties_solo](https://github.com/open-dynamic-robot-initiative/robot_properties_solo) repository. The stl meshes for visualization are also from the same repository. The copyright notice below applies to them. The collision meshes and additional xacro files are our own.

BSD 3-Clause License

Copyright (c) 2019, New York University and Max Planck Gesellschaft.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.