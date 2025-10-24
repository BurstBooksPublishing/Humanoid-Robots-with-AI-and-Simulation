#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import open3d as o3d

# helper: convert PointCloud2 to numpy xyz array (implementation omitted)
def pc2_to_xyz(msg): ...
# helper: transform points by pose T (4x4)
def transform_points(points, T): ...

class ObstacleNode:
    def __init__(self, node):
        self.node = node
        self.sub_lidar = node.create_subscription(PointCloud2,'/lidar',self.cb_lidar,10)
        self.sub_depth = node.create_subscription(PointCloud2,'/depth',self.cb_depth,10)
        self.pub_grid = node.create_publisher(OccupancyGrid,'/obstacle_grid',10)
        # parameters tuned from design constraints
        self.voxel_size = 0.03  # m
        self.grid_res = 0.05    # occupancy grid resolution
        self.grid_size = 200    # 10m x 10m grid

    def cb_lidar(self,msg):
        pts = pc2_to_xyz(msg)
        # motion compensation: transform based on timestamp and ego-pose (not shown)
        # apply voxel grid downsample
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pcd = pcd.voxel_down_sample(self.voxel_size)
        # remove ground via RANSAC plane fitting
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                 ransac_n=3, num_iterations=50)
        non_ground = pcd.select_by_index(inliers, invert=True)
        # project non_ground to occupancy grid and update log-odds
        self.update_occupancy(non_ground)

    def cb_depth(self,msg):
        pts = pc2_to_xyz(msg)
        # similar processing as lidar, with shorter range and denser points
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pcd = pcd.voxel_down_sample(self.voxel_size/2)
        pcd = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)[0]
        self.update_occupancy(pcd)

    def update_occupancy(self,pcd):
        # convert to numpy and update 2D grid occupancy (raycasting omitted)
        pts = np.asarray(pcd.points)
        # map xy to grid indices and update log-odds table (implementation detail)
        # publish OccupancyGrid when updated
        grid = OccupancyGrid()
        # fill grid.data from internal log-odds map
        self.pub_grid.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('obstacle_node')
    ObstacleNode(node)
    rclpy.spin(node)