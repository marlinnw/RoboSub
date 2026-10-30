

"""test_rail_follower.launch.py -- brings up the whole sub for a rail-following test.

    On the sub (Jetson, inside tmux):   ros2 launch computer_test test_rail_follower.launch.py
    In the simulator (laptop):          ros2 launch computer_test test_rail_follower.launch.py mavros:=false
    Also record a bag:                  ... record:=true

What it starts:
    MAVROS               link to the Cube (skipped with mavros:=false)
    ardusub_bridge       /cmd_effort -> RC override       (starts INACTIVE)
    controller_input     game controller -> /cmd_effort   (activated; hold the deadman to drive)
    camera_publisher     CSI camera                       (activated)
    lab_filter           color filter                     (activated)
    rail_detector        finds the rail                   (activated)
    rail_follower        steers along the rail            (NOT activated: press Select to start it)

Still done by hand, on purpose (these make the thrusters spin):
    1. QGroundControl: set the mode, then arm
    2. ros2 topic pub --once /ardusub_bridge/command std_msgs/String "{data: 'activate'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# >>> EDIT THESE ------------------------------------------------------------
# MAVROS / Cube
MAVROS_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
MAVROS_BAUD = 115200             # must match SERIAL2_BAUD on the Cube
LAPTOP_IP = '192.168.2.1'        # QGroundControl on the field laptop gets a copy of the MAVLink traffic

# Camera
CAMERA_DEVICE = 'csi0'           # 'csi0' = CSI ribbon camera, '0' = USB camera
CAMERA_FPS = 30
CAMERA_ROTATE = 90                # degrees clockwise (0, 90, 180, 270) -- set once you've checked the image
# <<< ------------------------------------------------------------------------

# A 90/270 rotation makes the image portrait (240 wide instead of 320); the follower needs to know
IMAGE_WIDTH = 240 if CAMERA_ROTATE in (90, 270) else 320


def send(topic, data, delay):
    """After `delay` seconds, publish one String command (waits for the node to be listening)."""
    return TimerAction(
        period=delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '-w', '1',
                 topic, 'std_msgs/msg/String', f"{{data: '{data}'}}"],
            output='screen',
        )],
    )


def generate_launch_description():
    use_mavros = LaunchConfiguration('mavros')
    record = LaunchConfiguration('record')

    return LaunchDescription([
        DeclareLaunchArgument('mavros', default_value='true',
                              description='Start MAVROS for the real Cube (false in the sim)'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record a bag of the main topics'),

        # ---------- link to the Cube ----------
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            output='screen',
            respawn=True,                       # restart automatically if it crashes
            respawn_delay=2.0,
            parameters=[{
                'fcu_url': f'{MAVROS_PORT}:{MAVROS_BAUD}',
                'gcs_url': f'udp://@{LAPTOP_IP}:14550',
                'system_id': 255,               # = SYSID_MYGCS on the Cube
                'tgt_system': 1,
                'tgt_component': 1,
            }],
            condition=IfCondition(use_mavros),
        ),
        Node(package='sub_py', executable='ardusub_bridge', output='screen'),   # stays inactive

        # ---------- driving ----------
        Node(
            package='sub_py',
            executable='controller_input',
            output='screen',
            parameters=[{'autonomy_command_topic': '/rail_follower/command'}],   # Select -> start the follower
        ),

        # ---------- vision pipeline ----------
        Node(package='sub_py', executable='camera_publisher', output='screen',
             parameters=[{'rotate': CAMERA_ROTATE}]),
        Node(package='sub_py', executable='lab_filter', output='screen'),
        Node(package='sub_py', executable='rail_detector', output='screen'),
        Node(package='sub_py', executable='rail_follower', output='screen',
             parameters=[{'image_width': IMAGE_WIDTH}]),

        # ---------- turn the nodes on, one at a time ----------
        send('/controller_input/command', 'activate', 2.0),
        send('/camera/command', f'activate {CAMERA_DEVICE} {CAMERA_FPS}', 4.0),
        send('/color_filter/lab/command', 'activate', 6.0),
        send('/rail_detector/command', 'activate', 8.0),
        # rail_follower is NOT activated here: press Start (release), then Select on the controller

        # ---------- optional bag ----------
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '/camera/image_raw/compressed',
                 '/rail_detector/image_annotated/compressed',
                 '/rail_detector/target',
                 '/cmd_effort',
                 '/control_command/active',
                 '/mavros/rc/override',
                 '/mavros/state'],
            output='screen',
            condition=IfCondition(record),
        ),
    ])