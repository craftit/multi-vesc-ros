
from launch import LaunchDescription
import launch_ros.actions

# ros2 launch realsense2_camera rs_launch.py
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# ros2 run ublox_gps ublox_gps_node

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "vesc", package='multi_vesc_driver', executable='multi_vesc_node', output='screen'),
    ])