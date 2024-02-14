from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument,
    IncludeLaunchDescription)
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration)
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Launch agruments
        DeclareLaunchArgument(name="robot", default_value="real",
                              description="Selects whether the robot will be simulated or real",
                              choices=["real", "sim"]),
        
        # Nodes to be launched
        # Sign detection
        Node(package="train_controller", executable="sign_detection",
             on_exit=actions.Shutdown()
             ),
        
        # Train controller
        Node(package="train_controller", executable="controller",
             on_exit=actions.Shutdown()
             ),
        
        # Interbotix node
        # IncludeLaunchDescription(launch_description_source="/home/alves/ws/interbotix/install/share/interbotix_xsarm_control/launch/xsarm_control.launch.py",
        #                          launch_arguments={
        #                             'robot_model': 'px100',
        #                             'hardware_type': 'fake'
        #                          }.items(),
        #                          condition=IfCondition(EqualsSubstitution(LaunchConfiguration("robot"), "sim"))),

        IncludeLaunchDescription(launch_description_source="/home/alves/ws/interbotix/install/share/interbotix_xsarm_control/launch/xsarm_control.launch.py",
                                 launch_arguments={
                                    'robot_model': 'px100',
                                    'hardware_type': 'actual'
                                 }.items(),
                                 condition=IfCondition(EqualsSubstitution(LaunchConfiguration("robot"), "real"))),  
    ])