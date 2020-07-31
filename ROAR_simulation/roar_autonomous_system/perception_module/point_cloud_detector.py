from ROAR_simulation.roar_autonomous_system.perception_module.detector import Detector
import logging
import open3d as o3d
import numpy as np
import cv2
import time
from typing import Optional


class PointCloudDetector(Detector):
    def __init__(self, sky_level=0.1, depth_scaling_factor=1000, **kwargs):
        super().__init__(**kwargs)
        self.sky_level = sky_level
        self.depth_scaling_factor = depth_scaling_factor
        self.logger = logging.getLogger("Point Cloud Detector")
        self.pcd: o3d.geometry.PointCloud = o3d.geometry.PointCloud()

        self.counter = 0

    def run_step(self) -> Optional[np.ndarray]:
        if self.counter % 10 == 0:
            points_3d = self.calculate_world_cords()
            # mean = np.mean(points_3d, axis=0)
            self.pcd.points = o3d.utility.Vector3dVector(points_3d)
            self.pcd.points = self.pcd.voxel_down_sample(0.05).points
            print(f"Curr Vehicle Location = {self.agent.vehicle.transform.location}")
            self.pcd.paint_uniform_color(color=[0, 0, 0])
            new_pcd: o3d.geometry.PointCloud = self.pcd.voxel_down_sample(1)
            new_pcd.estimate_normals(fast_normal_computation=True)
            pcd_tree = o3d.geometry.KDTreeFlann(new_pcd)
            print("old_pcd_size:", self.pcd)
            print("sampled_pcd_size", new_pcd)

            [k, idx, _] = pcd_tree.search_knn_vector_3d(self.agent.vehicle.transform.location.to_array(),
                                                        knn=1000)
            np.asarray(new_pcd.colors)[idx[1:], :] = [0, 0, 1]
            o3d.visualization.draw_geometries([new_pcd])
            print("found_neighbor", len(idx))
            print()

        self.counter += 1
        return None

    def calculate_world_cords(self):
        depth_img = self.agent.front_depth_camera.data
        # get a 2 x N array for their indices
        ground_loc = np.where(depth_img < self.sky_level)
        depth_val = depth_img[depth_img < self.sky_level] * self.depth_scaling_factor
        ground_loc = ground_loc * depth_val

        # compute raw_points
        raw_points = np.vstack([ground_loc, depth_val])

        # convert to cords_y_minus_z_x
        cords_y_minus_z_x = np.linalg.inv(self.agent.front_depth_camera.intrinsics_matrix) @ raw_points

        # convert to cords_xyz_1
        ones = np.ones((1, np.shape(cords_y_minus_z_x)[1]))

        cords_xyz_1 = np.vstack([
            cords_y_minus_z_x[2, :],
            cords_y_minus_z_x[0, :],
            -cords_y_minus_z_x[1, :],
            ones
        ])
        extrinsics_matrix = \
            self.agent.front_depth_camera.transform.get_matrix() @ self.agent.vehicle.transform.get_matrix()
        # multiply by cam_world_matrix
        points_3d = (extrinsics_matrix @ cords_xyz_1)[:3, :].T  # i have all points now
        return points_3d
