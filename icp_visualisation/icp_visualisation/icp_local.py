import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import icp_visualisation.ros2_numpy.point_cloud2 as pcl2 #icp_localiser.ros2_numpy
import pcl
import collections, operator
#import sensor_msgs_py.point_cloud2 as pcl2

class pose():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z=  0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

class IcpLocal(Node):
    def __init__(self):
        super().__init__('icp_local')
        ### Parameters
        self.prev_acc = 0.0
        self.curr_acc = 0.0
        self.mapLoaded = 0
        #self.prev_imu_time
        #self.curr_imu_time
        self.currentPCL = 0
        self.numberOfPoints = 0
        #self.icp = pcl.IterativeClosestPoint()
        #self.icp = pcl.registration.IterativeClosestPoint()
        #self.icp = pcl.re

        #### Poses ####
        self.initialPose= pose()
        #self.icpPose, 
        self.currentPose = pose()
        self.previousPose = pose()
        self.predictPose = pose()
        self.staticMap = pcl.load('/home/mcav/liam_ws/localisation/map.pcd')
        self.staticMapArray  = 0
        self.cloudIn = pcl.PointCloud()
        self.cloudOut = pcl.PointCloud()
        self.icp = 0
        
        self.loadMap()
        
        #### Subscribers ####
        self.lidar_callback = self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar', self.lidar_callback, 10)
        #self.imu_callback = self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.imu_callback, 10)
        #self.gnss_callback = self.create_subscription(NavSatFix, '/carla/ego_vehicle/gnss', self.gnss_callback, 10)
    
    def loadMap(self):
        #mapArr = pcl.load_XYZRGBA(self.staticMap)
        #mapxyzArr = pcl2.get_xyz_points(mapArr)
        print("loading the map: \n")
        print(self.staticMap.to_array())
        self.staticMapArray = self.staticMap.to_array()
        self.mapLoaded = 1
        #self.staticMap.data = np.ndarray.tobytes(self.pclData.to_array())
        #print(np.array(list(read_(self.pclData))))
        # map = open('/home/mcav/liam_ws/localisation/map.pcd')
        # self.staticMap.data.fromfile(map, 1000)

    def lidar_callback(self, data):
        #assert isinstance(data, PointCloud2)
        #gen = _point_cloud2()
        # height = 1 (unordered) width = 8610 (length of point cloud)
        # 
        #data.read
        if self.mapLoaded ==1:
            pcArr = pcl2.pointcloud2_to_array(data)
            xyzArr = pcl2.get_xyz_points(pcArr)
            # Fix later
            self.predictPose = self.initialPose
            self.currentPCL = xyzArr.astype(np.float32)
            print(type(xyzArr))
            self.numberOfPoints = len(self.currentPCL)
            #make_ICP = self.icp.make_IterativeClosestPoint()
            #print("MapLoaded")
            #print(dir(self.staticMap.make_IterativeClosestPoint()))
            #print(type(self.icp))
            self.cloudIn.from_array(self.currentPCL)
            self.cloudOut.from_array(self.staticMapArray)
            self.icp = self.cloudIn.make_IterativeClosestPoint()
            converged, transf, estimate, fitness = self.icp.icp(self.cloudIn, self.cloudOut)
            print("Coverged: ", converged, " Trasnf: ", transf, " Estimate; ", estimate, " Fitness", fitness)

            print("lidar: \n")
            print(xyzArr)
        else:
            print("Do not have reference map")


    def imu_callback(self, msg):
        print('getting data2')
        ## Find initial position?
        pass

    def gnss_callback(self, msg):
        print('getting data3')
        ## Find initial position?
        pass


def main(args=None):
    rclpy.init(args=args)
    IcpNode = IcpLocal()
    print('Hi from icp_visualisation.')
    # lidar is sensor_msgs/msg/PointCloud2
    try:
        rclpy.spin(IcpNode)
    except KeyboardInterrupt:
        print("Stopping ICP Node")
    #except BaseException:
     #   print("Exception in server: ", file=sys.stderr)
      #  raise
    finally:
        IcpNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

