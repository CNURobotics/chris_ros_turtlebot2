CHRISLab Turtlebot2 System
==========================

## Introduction

Custom models and launch setup for CHRISLab Kobuki-based Turtlebot 2 system.

This repository contains code that interfaces with the ROS 2 versions of the
[Kobuki Turtlebot2] models.

Installation and Setup
----------------------

This package has a number of dependencies.  

We will add a number of repos to our setup and install.

Quickly skim this README before installing or running anything:

This demonstration makes use of the following repositories:

<pre>
- git: {local-name: src/chris_ros_turtlebot2,     uri: 'https://github.com/CNURobotics/chris_ros_turtlebot2.git',     version: ros2-devel }
</pre>

At this current stage, some Kobuki Turtlebot2 related packages are not released in ROS2 binary form, so we are using the following:
<pre>
https://github.com/kobuki-base/kobuki_ros_interfaces.git devel
https://github.com/stonier/ecl_tools.git                 devel
https://github.com/stonier/sophus.git                    release/1.2.x
https://gitlab.pcs.cnu.edu/zutell_thesis/ecl_lite.git    humble-test (Custom fork!)
https://gitlab.pcs.cnu.edu/zutell_thesis/ecl_core.git    humble-test (Custom fork!)
https://gitlab.pcs.cnu.edu/zutell_thesis/kobuki_ros.git  humble-test (Custom fork!)
https://github.com/CNURobotics/openni2_camera.git        debug-astra-galactic
</pre>


Install in the `src` folder of your WORKSPACE_ROOT, and from the

<pre>
colcon build
. setup.bash
</pre>

> NOTE: Anytime you build new packages, you need to re-run the setup.bash script inside the workspace root.  
> Anytime you change a Python script or launch file, you need to re-run `colcon build` from the WORKSPACE_ROOT folder, but you only need to re-source `. setup.bash` when the package information and folders change.


## Operation
---------

### Start the hardware
`sudo chmod a+rw /dev/ttyACM0`
  * This allows for the Hokuyo LiDAR on the Turtlebot2 to get LiDAR data
  * Best to set this using a udev.rule (Google)

`ros2 launch chris_ros_turtlebot2 turtlebot_bringup.launch.py`
  * Starts the hardware on the Turtlebot2  

  > NOTE: The below launch files are configured to default to use_sim_time:=True
  > For each , add the option to set use_sim_time:=False.
  > The on robot setup, has been modified to do this for you.

or

### Start Gazebo with the desired world model.
 * See `chris_world_models` repository for some available options
   * e.g. `ros2 launch chris_world_models gazebo_creech_world.launch.py`

   * Start the simulated robot
       * `ros2 launch chris_ros_turtlebot2 turtlebot_gazebo.launch.py`
       * Starts the simulated Kobuki Turtlebot2 in Gazebo with the CHRISLab sensor configuration


## Optional: Explore URDF
---

The `robot_description` is an XML-file used to describe the robot model.
This is loaded by the launch of the robot.

Our CHRISLab robot uses a Hokuyo URG-04 single plane laser scanner and a Orbec Astra camera.

The model separates the physical properties and visual properties.  A `collision` definition is
used to model shapes that interact, while `visual` define colors and textures that are shown in 3D view.
The physics is governed by the `inertial` properties (e.g. mass and intertia about different axes).
The movements are governed by `joints` that define axes of movement between links and any friction modeled by `damping`.

This `robot_description` is used by RViz for viewing robot, Gazebo for physics based simulations, and for collision checking in planners.

See http://gazebosim.org/tutorials/?tut=ros_urdf for a discussion of URDF in Gazebo

---
[ROS 2]: https://docs.ros.org/en/foxy/index.html
[Kobuki Turtlebot2]: https://github.com/kobuki-base/kobuki_ros
[Kobuki ROS]: https://github.com/kobuki-base/kobuki_ros
[Koubki ROS Interfaces]: https://github.com/kobuki-base/kobuki_ros_interfaces

[ROS 2 Installation]: https://docs.ros.org/en/humble/Installation.html
