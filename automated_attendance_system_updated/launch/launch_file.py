from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Node 4: Batch Image Subscriber (For face recognition)
        Node(
            package='project',
            executable='photo_check_sub',
            name='photo_check_sub',
            output='screen'
        ),
        Node(
            package='project',
            executable='attendence_monitor',
            name='attendence_monitor',
            output='screen'
        ),


        # Node 5: Output Feed (Displays recognized student)
    
        # Node(
        #     package='project',
        #     executable='check_attendence',
        #     name='check_attendence',
        #     output='screen'
        # ),
        Node(
            package='project',
            executable='student_display',
            name='student_display',
            output='screen'
        ),
    ])
