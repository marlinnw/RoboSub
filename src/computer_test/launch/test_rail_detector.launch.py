"""Rail-detection pipeline for desk testing (laptop, USB camera).

Pipeline:
    camera_publisher -> lab_filter -> rail_detector

Point the camera at a rail-colored object and watch
/rail_detector/target or /rail_detector/image_annotated to tune the
filter and detector.

Every node starts inactive. The timers below send each one "activate",
upstream first.

Run:  ros2 launch computer_test test_rail_detector.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# "activate <device> <fps>" for camera_publisher: USB camera /dev/video0 at 30 fps
CAMERA_COMMAND = 'activate 0 30'

def send(topic, text):
    """Launch action that publishes one std_msgs/String on topic, then exits.

    '-w 1' waits for at least one subscriber first, so the message isn't
    lost if the target node is still starting up.
    """
    return ExecuteProcess(
            cmd = ['ros2', 'topic', 'pub', '--once', '-w', '1',
                    topic, 'std_msgs/msg/String', f"{{data: '{text}'}}"],
                    output='log')



def generate_launch_description():
    """Start the vision nodes, then activate them in pipeline order."""
    return LaunchDescription([
        Node(package='sub_py', executable='camera_publisher', output='screen'),
        Node(package='sub_py', executable='lab_filter', output='screen'),
        Node(package='sub_py', executable='rail_detector', output='screen'),

        # Staggered activation (seconds after launch)
        TimerAction(period=1.0, actions=[send('/camera/command', CAMERA_COMMAND)]),
        TimerAction(period=2.0, actions=[send('/color_filter/lab/command', 'activate')]),
        TimerAction(period=3.0, actions=[send('/rail_detector/command', 'activate')])
    ])
