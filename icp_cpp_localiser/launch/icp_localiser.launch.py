import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
   use_sim_time = LaunchConfiguration('use_sim_time', default='true')

   config = os.path.join(
      get_package_share_directory('icp_cpp_localiser'),
      'config',
      'icp_localiser.yaml'
      )

   return LaunchDescription([
      DeclareLaunchArgument(
         'use_sim_time',
         default_value='true',
         description='Use simulation (Carla) clock if true'),
      Node(
         package='icp_cpp_localiser',
         executable='icp_localiser_node',
         name='icp_localiser_node',
         parameters=[config, {'use_sim_time': use_sim_time}]
      )
   ])