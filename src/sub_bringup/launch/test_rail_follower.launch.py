from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

CAMERA_COMMAND = 'activate csi0 30'

def send(topic, text):
    return ExecuteProcess(
            cmd = ['ros2', 'topic', 'pub', '--once', '-w', '1',
                    topic, 'std_msgs/msg/String', f"{{data: '{text}'}}"],
                    output='log')



def generate_launch_description():
    return LaunchDescription([
        Node(package='joy',     executable='game_controller_node',  output='screen`'),
        Node(package='sub_py',  executable='controller_input',      output='screen',
             parameters=[{'autonomy_command_topic': '/rail_follower/command'}]),
        Node(package='sub_py',  executable='camera_publisher',      output='screen',
             parameters=[{'rotate': 90}]),
        Node(package='sub_py',  executable='lab_filter',            output='screen'),
        Node(package='sub_py',  executable='rail_detector',         output='screen'),
        Node(package='sub_py',  executable='rail_follower',         output='screen'),
        Node(package='sub_py',  executable='ardusub_bridge',        output='screen'),

        TimerAction(period=1.0,     actions={send('/controller_input/command',  'activate')}),
        TimerAction(period=3.0,     actions=[send('/camera/command',            CAMERA_COMMAND)]),
        TimerAction(period=5.0,     actions=[send('/color_filter/lab/command',  'activate')]),
        TimerAction(period=7.0,     actions=[send('/rail_detector/command',     'activate')]),
        TimerAction(period=9.0,     actions=[send('/rail_follower/command',     'activate')]),
        TimerAction(period=11.0,    actions= [send('/ardusub_bridge/command',   'activate')])
    ])