from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    student_node = Node(
        package='my_student_pkg',
        executable='student_node',
        name='student_node',
        parameters=[{
            'max_voltage': 42.0,
            'min_voltage': 36.0
        }]
    )

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', 'battery_bag'],
        output='screen'
    )

    return LaunchDescription([
        student_node,
        bag_record
    ])