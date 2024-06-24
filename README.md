# multi-vesc-ros

NOTE: This is work in progress, it is not yet functional

Controlling multiple vescs over CAN from ROS



# Notes
    colcon build --event-handlers console_cohesion+
#    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
    source install/setup.bash
    ros2 launch multi_vesc multi_vesc.launch.py
