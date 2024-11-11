from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    rqt_image_view=ExecuteProcess(
        cmd=[[
            'ros2 ','run ','rqt_image_view ' ,'rqt_image_view'
        ]],

        shell=True,
        output='screen',
    )
    
    
    return LaunchDescription([
        TimerAction(period=1.0, actions=[rqt_image_view]),

        Node(
            package='ptp_move', # 패키지 이름
            executable='object_detection_node', # 실행 파일 이름
            name='object_detection_node',
            output='screen'
        ),

    ])
