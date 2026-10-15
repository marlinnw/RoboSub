from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

CAMERA_COMMAND = 'activate 0 30'

def send(topic, text):
    return ExecuteProcess(
            cmd = ['ros2', 'topic', 'pub', '--once', '-w', '1',
                    topic, 'std_msgs/msg/String', f"{{data: '{text}'}}"],
                    output='log')



def generate_launch_description():
    return LaunchDescription([
        Node(package='sub_py', executable='camera_publisher', output='screen'),
        Node(package='sub_py', executable='lab_filter', output='screen'),
        Node(package='sub_py', executable='rail_detector', output='screen'),

        TimerAction(period=1.0, actions=[send('/camera/command', CAMERA_COMMAND)]),
        TimerAction(period=2.0, actions=[send('/color_filter/lab/command', 'activate')]),
        TimerAction(period=3.0, actions=[send('/rail_detector/command', 'activate')])
    ])