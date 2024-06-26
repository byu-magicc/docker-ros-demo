# Docker ROS Demo

This repository includes template files and instructions for completing a basic ROS2 demonstration with ROSflight and ROSplane, run inside Docker containers. The goal is to be able to run ROSflight inside a Docker container and code up a simple path-follower controller using the template code.

This guide is written with instructions current for July 2024. If reading this in the far future, refer to the docs in the additional info for up-to-date information.

## Why ROS and Docker instead of a native ROS installation?

Although ROS is a great tool, it isn't without its downsides. One of those downsides is that it is a huge software package that is very difficult to fix if it ever breaks on your system, with reinstalling your OS often being the easiest solution to fixing it. Another major problem is ROS is very inflexible about which versions of ROS can be ran on which operating systems, making it difficult to develop with multiple versions of ROS at the same time or for developing with ROS on an unsupported platform. Docker is the solution to both of these problems. Docker puts all the software and dependencies required to run ROS inside an isolated container that can be ran on pretty much any system. This means you can download and use any version of ROS on any OS easily without ever needing to actually install it on your system. And if anything ever breaks, you can recreate the container in seconds and start with a brand new system.

## Installing Docker

If you use Windows or MacOS then you will need to install [Docker Desktop](https://docs.docker.com/desktop/). If you use Linux, you can also install Docker Desktop, but if you prefer CLI tools over GUI tools (like us) then install [Docker Engine](https://docs.docker.com/engine/). Note that if you install Docker Engine, you will either need to use `sudo` for every docker command or add yourself to the docker group using the [post-installation Linux instructions](https://docs.docker.com/engine/install/linux-postinstall/).

## Running GUIs inside of Docker containers

Running Docker in a headless environemnt is easy and should work out of the box. However, many ROS GUI tools like Gazebo, rqt_graph, and plotjuggler do not work in a headless environment. Fortunately GUIs can be used within Docker containers, but the setup is a little more complicated and machine specific. The Dockerfile and compose.yaml files included in this repo are set up to be able to display GUIs using a X11 windowing server. Most Linux distributions are compatible with the X11 protocol either through native X11 or Xwayland. Windows 11 should also have support through WSL. MacOS doesn't have native support, but a X server can be installed with [XQuarts](https://www.xquartz.org/).

To enable GUIs with this Dockerfile and compose.yaml file, switch the base image in the Dockerfile to use `osrf/ros:${ROS_DISTRO}-desktop-full` instead of `ros:${ROS_DISTRO}` and uncomment any lines within the compose.yaml file that specify being needed for GUIs.

TODO: At the time of writing, GUIs have only been tested on Linux. If you get this working on a Windows or MacOS machine, please submit a pull request to update this guide with instructions on how to replicate your work.

## Running the demo code

### Docker Engine instructions

First, navigate to the directory with the compose.yaml file. Any `docker compose` commands need to be run from the directory where the compose.yaml file is.

To build the Docker image from which we will create the container to run ROSflight and ROSplane, use the command `docker compose build`. To start the container in the background, run `docker compose up -d`. To enter the container with a terminal, use `docker compose exec rosflight_demo bash`. You can enter the container with as many terminals as you need.

### Docker Desktop instructions

TODO: Someone with Docker Desktop should figure out how to do this.

### Running ROSflight and ROSplane

To start the ROSflight simulation, use the command `ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=anaconda gui:=false` (remove the gui=false flag if you have GUIs enabled). Since ROSflight runs the exact same code in simulation as it does in real life, we need to set up the firmware and allow for control from ROSplane. Open a new termial in the Docker container and set the firmware parameters with `ros2 launch rosflight_sim fixedwing_init_firmware.launch.py`. Wait for that to complete. Arm the controller and disable RC override with these commands `ros2 service call /toggle_arm std_srvs/srv/Trigger && ros2 service call /toggle_override std_srvs/srv/Trigger`. Now, launch ROSplane with `ros2 launch rosplane_sim sim.launch.py aircraft:=anaconda`. If everything worked, then the MAV in the simulation should start flying!

For more detailed information (if you run into problems) on using ROSplane and ROSflight, see [the ROSflight docs](https://docs.rosflight.org/git-main/).

To make sure the MAV is actually moving in a headless environment, you can echo topics to see what the MAV is doing. Use `ros2 topic list` to show all avaliable topics, and then use `ros2 topic echo /topic_name` to see what is being published.

### Run custom ROSplane package

TODO: Once template code has been written, add instructions on how to use it here.

## Additional info

This isn't intended to be an all inclusive guide but rather a basic example to get you started quickly and experimenting with ROS and Docker. Check out these resources if you want to learn more.
- [ROS Docs](https://docs.ros.org/en/humble/)
- [ROSflight Docs](https://docs.rosflight.org/git-main/) (use the git-main version until v2.0 has been released)
- [Short guide for using ROS in docker](https://docs.rosflight.org/git-main/user-guide/ros2-setup/#using-a-docker-container-to-run-ros2)
- [Docker Docs](https://docs.docker.com/)
