import rclpy
from rclpy.node import Node
# imports omitted: message types, odometry, pointcloud, GTSAM wrapper
class PoseGraphUpdater(Node):
    def __init__(self):
        super().__init__('pose_graph_updater')
        # subscribe to sensors (callbacks do buffering and timestamping)
        self.create_subscription(ImuMsg,'/imu/data', self.imu_cb, 200)
        self.create_subscription(PointCloud2,'/camera/depth/points', self.rgbd_cb, 10)
        self.create_subscription(PointCloud2,'/velodyne/points', self.lidar_cb, 10)
        # initialize pose graph optimizer (iSAM2/GTSAM)
        self.optimizer = GTSAMWrapper()  # lightweight wrapper
    def imu_cb(self,msg):
        self.buffer_imu(msg)  # high-rate preintegration buffer
    def rgbd_cb(self,msg):
        odom = run_rgbd_odometry(msg)  # returns relative pose
        self.optimizer.add_odometry_factor(odom)  # add visual factor
        self.try_optimize()
    def lidar_cb(self,msg):
        rel = scan_match(msg)  # ICP relative pose
        self.optimizer.add_geometry_factor(rel)  # add LiDAR factor
        if loop_candidate(rel):
            self.optimizer.add_loop_factor(loop_constraint(rel))
        self.try_optimize()
    def try_optimize(self):
        self.optimizer.update()  # incremental optimizer step
        pose = self.optimizer.get_current_pose()
        self.publish_pose(pose)
# node spin omitted