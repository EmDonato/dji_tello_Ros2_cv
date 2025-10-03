from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='game_controller',
            parameters=[{
                'device_id': 1,
                'deadzone': 0.05,
                'autorepeat_rate': 0.0,
                'sticky_buttons': False,
                'coalesce_interval_ms': 1
            }],
            output='screen'
        ),
        Node(
            package='dji_tello_ros2',
            executable='gamepad_node',
            name='gamepad_tello_node',
            output='screen'
        ),
        Node(
            package='dji_tello_ros2',
            executable='tello_driver_node',
            name='dji_tello_driver',
            output='screen'
        ),
        Node(
            package='dji_tello_yolo',          
            executable='yolo_person_node',     
            name='tello_yolo',
            output='screen'
        ),
        Node(
            package='dji_tello_aruco',          
            executable='tello_aruco_node',     
            output='screen'
        )
    ])

