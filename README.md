# irob-saf ROS 2 Components

## About
This library contains ROS 2 ports and extensions for the ABC-iRobotics Surgical Automation Framework ([irob-saf](https://github.com/ABC-iRobotics/irob-saf)). Compatible with Ubuntu on Xorg.

## Installation
These components require the original irob-saf stack to be installed, alongside a ROS 2 version. As such, installation through Docker is highly recommended. Build the image by running the following:
 
    docker build -t irob-saf:latest --build-arg BUILDKIT_CONTEXT_KEEP_GIT_DIR=1 https://github.com/ABC-iRobotics/irob-saf.git#foxy_bridge

Create a container from the freshly created image by running:

    docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged -v /dev:/dev --name irob-saf irob-saf

The above command passes through all hardware devices. For improve security, pass through only the necessary folders in `/dev`, such as `fw0` through `fw2` for the dVRK, alongside any USB devices used.

Using graphical applications requires Xorg access to be granted to the container.

    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' irob-saf`

Then onwards, the container can be launched by running this command:
    docker start -ai irob-saf

## Usage
Generally, multiple shells will be needed to launch these nodes, running either ROS1, ROS2 or both. These can be sourced by running `/ros_entrypoint.sh` with either `ros1`, `ros2` or `bridge` as an argument. To ease this, a configuration for the Terminator terminal emulator is provided. This can be added under `[layouts]` in your Terminator config, which is normally `~/.config/terminator/config`.

<details>
  <summary>4x2 layout</summary>

    [[irob-saf]]
    [[[root]]]
      type = Window
      parent = ""
      size = 1920, 1080
    [[[grandgrand]]]
      type = HPaned
      parent = root
    [[[grandleft]]]
      type = VPaned
      parent = grandgrand
    [[[leftleft]]]
      type = VPaned
      parent = grandleft
    [[[term1]]]
      profile = default
      type = Terminal
      order = 0
      parent = leftleft
      command = docker exec -it irob-saf bash
    [[[term2]]]
      profile = default
      type = Terminal
      order = 1
      parent = leftleft
      command = docker exec -it irob-saf bash
    [[[leftright]]]
      type = VPaned
      parent = grandleft
    [[[term3]]]
      profile = default
      type = Terminal
      order = 0
      parent = leftright
      command = docker exec -it irob-saf bash
    [[[term4]]]
      profile = default
      type = Terminal
      order = 1
      parent = leftright
      command = docker exec -it irob-saf bash
    [[[grandright]]]
      type = VPaned
      parent = grandgrand
    [[[rightleft]]]
      type = VPaned
      parent = grandright
    [[[term5]]]
      profile = default
      type = Terminal
      order = 0
      parent = rightleft
      command = docker exec -it irob-saf bash
    [[[term6]]]
      profile = default
      type = Terminal
      order = 1
      parent = rightleft
      command = docker exec -it irob-saf bash
    [[[rightright]]]
      type = VPaned
      parent = grandright
    [[[term7]]]
      profile = default
      type = Terminal
      order = 0
      parent = rightright
      command = docker exec -it irob-saf bash
    [[[term8]]]
      profile = default
      type = Terminal
      order = 1
      parent = rightright
      command = docker exec -it irob-saf bash
</details>

Afterwards, you can run the following script to launch Terminator in the above configuration:

    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' irob-saf`
    docker start irob-saf
    terminator -l irob-saf

Make sure to re-compile and re-source after modifications!

    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash

See the READMEs in each package for instructions on individual workflows.