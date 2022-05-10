import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class PathCreationNode(Node):
    def __init__(self):
        super().__init__('icp_path')
        self.icp_subscriber = self.create_subscription(
                            PoseWithCovarianceStamped,'/my_icp_pose', 
                            self.odom_cb, 1)
        self.icp_subscriber # prevent 'unused variable' warning
        self.path_publisher = self.create_publisher(Path, '/path', 1)
        self.path = Path()

    def odom_cb(self,data):
        self.path.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

def main(args = None):
    rclpy.init(args=args)
    pathnode = PathCreationNode()
    rclpy.spin(pathnode)
    pathnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Printing path of icp")
    main()