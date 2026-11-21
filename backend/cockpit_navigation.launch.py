"""Nav2 Humble, explicit standalone nodes and gated velocity output."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root=Path(__file__).resolve().parent
    params=str(root/'config/nav2_params.yaml')
    controller=str(root/'config/nav2_controller.yaml')
    tf=[('/tf','tf'),('/tf_static','tf_static')]
    drivers=[]
    for package,file in [('controller','controller.launch.py'),('peripherals','lidar.launch.py'),('peripherals','depth_camera.launch.py')]:
        drivers.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package))/'launch'/file))))
    nodes=[
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
             parameters=[params,{'yaml_filename':LaunchConfiguration('map')}],remappings=tf),
        Node(package='nav2_amcl',executable='amcl',name='amcl',output='screen',parameters=[params],remappings=tf),
    ]
    specs=[('nav2_controller','controller_server',[params,controller],[('cmd_vel','/cockpit/nav_raw')]),
           ('nav2_smoother','smoother_server',params,[]),('nav2_planner','planner_server',params,[]),
           ('nav2_behaviors','behavior_server',params,[('cmd_vel','/cockpit/nav_raw')]),
           ('nav2_bt_navigator','bt_navigator',params,[]),('nav2_waypoint_follower','waypoint_follower',params,[]),
           ('nav2_velocity_smoother','velocity_smoother',params,
            [('cmd_vel','/cockpit/nav_raw'),('cmd_vel_smoothed','/cockpit/nav_cmd_vel')])]
    for package,executable,config,remaps in specs:
        nodes.append(Node(package=package,executable=executable,name=executable,output='screen',
                          parameters=config if isinstance(config,list) else [config],remappings=tf+remaps))
    for suffix,names in [('localization',['map_server','amcl']),('navigation',[s[1] for s in specs])]:
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
            name='lifecycle_manager_'+suffix,output='screen',
            parameters=[{'autostart':True,'use_sim_time':False,'node_names':names,'bond_timeout':10.0}]))
    return LaunchDescription([DeclareLaunchArgument('map'),DeclareLaunchArgument('start_drivers',default_value='true'),
        GroupAction(drivers,condition=IfCondition(LaunchConfiguration('start_drivers'))),*nodes])
