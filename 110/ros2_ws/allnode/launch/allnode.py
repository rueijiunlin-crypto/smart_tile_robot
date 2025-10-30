import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rectangles_detect',
            executable='rectangles_detect.py',
            name='rectangles_detect_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/World_Coordinates', '/World_Coordinates')
            ]
        ),
         Node(
            package='sound_record',
            executable='sound_record.py',
            name='sound_record_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/moveknock_locate_node', '/moveknock_locate_node'),
                ('/move2', '/move2')  # Added remapping for /move2
            ]
        ),
        Node(
            package='top_base_notify',
            executable='top_base_noti.py',
            name='top_base_notify_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/tile_center', '/tile_center'),
                ('/camera/pose', '/camera/pose'),
                ('/move', '/move'),
                ('/move2', '/move2'),
                ('/move_hit_initialized', '/move_hit_initialized')
            ]
        ),
        Node(
            package='manual',
            executable='manual.py',
            name='manual',
            output='screen',
            parameters=[],
            remappings=[        
                ('/keyboard_input', '/keyboard_input')
            ]
        )
    ])  # Make sure this closing bracket is present

