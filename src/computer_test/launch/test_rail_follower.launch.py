"""Full rail-following stack for desk testing (laptop, USB camera).

Same as sub_bringup/test_rail_follower.launch.py but uses USB camera
/dev/video0 instead of the Jetson CSI camera, with no image rotation.

Pipeline:
    joy -> controller_input --+
                              +--> /cmd_effort -> ardusub_bridge -> MAVROS
    camera_publisher -> lab_filter -> rail_detector -> rail_follower --+

Every node starts inactive. The timers below send each one "activate" a
couple of seconds apart, upstream first; the bridge goes last.

Run:  ros2 launch computer_test test_rail_follower.launch.py
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
    """Start every node, then activate them in pipeline order."""
    return LaunchDescription([
        Node(package='sub_py', executable='controller_input',   output='screen',
             parameters=[{'autonomy_command_topic': '/rail_follower/command'}]),
        Node(package='sub_py', executable='camera_publisher',   output='screen'),
        Node(package='sub_py', executable='lab_filter',         output='screen'),
        Node(package='sub_py', executable='rail_detector',      output='screen'),
        Node(package='sub_py', executable='rail_follower',      output='screen'),
        Node(package='sub_py', executable='ardusub_bridge',     output='screen'),

        # Staggered activation (seconds after launch)
        TimerAction(period=1.0,     actions=[send('/controller_input/command', 'activate')]),
        TimerAction(period=3.0,     actions=[send('/camera/command',            CAMERA_COMMAND)]),
        TimerAction(period=5.0,     actions=[send('/color_filter/lab/command',  'activate')]),
        TimerAction(period=7.0,     actions=[send('/rail_detector/command',     'activate')]),
        TimerAction(period=9.0,     actions=[send('/rail_follower/command',     'activate')]),
        TimerAction(period=11.0,    actions=[send('/ardusub_bridge/command',   'activate')])
    ])
