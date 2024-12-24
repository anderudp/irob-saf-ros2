# irob_vision_support
Computer vision utilities for surgical automation.

## Cylindrical marker hand-eye registration
First, launch the dVRK:

**Shell 1**

    /ros_entrypoint.sh ros1
    roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=$HOME/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/OE-daVinci/console-PSM1.json

Then, launch the ROS 1-ROS 2 bridge:

**Shell 2**

    /ros_entrypoint.sh bridge
    ros2 run ros1_bridge dynamic_bridge

Run the cylmarker detector node:

**Shell 3**

    /ros_entrypoint.sh ros2
    ros2 run irob_vision_support cylmarker_detector

Capture camera frames with the marker in vision and attempt registration:

**Shell 4**

    /ros_entrypoint.sh ros2
    ros2 launch irob_vision_support hand_eye_registration_launch.py

If attempts to locate the marker fail continuously, the detection color ranges may need to be adjusted. Using the images taken in the previous step, this can be done using a graphical utility.

**Shell 5**

    python3 irob-saf-ros2/irob_vision_support/cylmarker_utils/pose_estimation/adjust_hsv.py

Find an appropriate combination, and then override the values in `~/ros2_ws/irob‑saf‑ros2/irob_vision_support/config/cylmarker/config.yaml`, then the registrator node can be re-launched.