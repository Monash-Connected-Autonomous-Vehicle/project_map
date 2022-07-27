import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
import carla
from carla import World, Transform, Location

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('icp_cpp_localiser'),
      'config',
      'icp_localiser.yaml'
      )

   return LaunchDescription([
      Node(
         package='icp_cpp_localiser',
         executable='icp_localiser_node',
         name='icp_localiser_node',
         parameters=[config]
      )
   ])