from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# 新增：导入 Node 类（如果用 Node 方式启动）
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 1. 启动 xArm7 MoveIt 仿真环境（包含 RViz 可视化）
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "xarm7_moveit_fake.launch.py",
                ]
            )
        ),
    )

    # 2. 启动你的运动脚本（推荐用 ExecuteProcess 方式，适配 Python 脚本）
    shape_drawer_node = ExecuteProcess(
        cmd=['python3', 
             os.path.join(
                 FindPackageShare("avatar_challenge").find("avatar_challenge"),
                 'src', 'shape_drawer.py'
             )],
        name='xarm_shape_drawer',
        output='screen',
        cwd=[FindPackageShare("avatar_challenge").find("avatar_challenge")]
    )

    # 【可选】如果想启动官方画正方形脚本，替换上面的 node 为这个：
    # square_official_node = ExecuteProcess(
    #     cmd=['python3', 
    #          os.path.join(
    #              FindPackageShare("avatar_challenge").find("avatar_challenge"),
    #              'src', 'xarm_square_official.py'
    #          )],
    #     name='xarm_square_drawer',
    #     output='screen',
    #     cwd=[FindPackageShare("avatar_challenge").find("avatar_challenge")]
    # )

    # 3. 返回完整的启动描述
    return LaunchDescription(
        [
            xarm_moveit_launch,  # 先启动可视化环境
            shape_drawer_node    # 再启动运动脚本（替换成 square_official_node 可跑官方脚本）
        ]
    )
