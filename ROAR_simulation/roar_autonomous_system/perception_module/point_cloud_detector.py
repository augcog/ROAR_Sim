from ROAR_simulation.roar_autonomous_system.perception_module.detector import Detector
import logging
import open3d as o3d
import numpy as np
import cv2
import time
from typing import Optional


class PointCloudDetector(Detector):
    def __init__(self, max_detectable_distance=0.1, depth_scaling_factor=1000, **kwargs):
        """

        Args:
            max_detectable_distance: maximum detectable distance in km
            depth_scaling_factor: scaling depth back to world scale. 1000 m = 1 km
            **kwargs:
        """
        super().__init__(**kwargs)
        self.max_detectable_distance = max_detectable_distance
        self.depth_scaling_factor = depth_scaling_factor
        self.logger = logging.getLogger("Point Cloud Detector")
        self.pcd: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.counter = 0

    def run_step(self) -> Optional[np.ndarray]:
        points_3d = self.calculate_world_cords(max_points_to_convert=10000)
        ground_indicies = np.where(points_3d[:, 0] > self.agent.vehicle.transform.location.to_array()[0])
        ground_points = points_3d[ground_indicies]
        self.pcd.points = o3d.utility.Vector3dVector(ground_points)

        if self.counter == 0:
            self.vis.add_geometry(self.pcd)
        else:
            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            ctrl: o3d.visualization.ViewControl = self.vis.get_view_control()
            ctrl.set_lookat(self.pcd.get_center())
            ctrl.set_front(self.pcd.get_center())
            # ctrl.set_up(self.agent.vehicle.transform.location.to_array())
            self.vis.update_renderer()


        self.counter += 1
        return None

    def calculate_world_cords(self, max_points_to_convert=5000):
        depth_img = self.agent.front_depth_camera.data
        # get a 2 x N array for their indices
        ground_loc = np.where(depth_img < self.max_detectable_distance)
        depth_val = depth_img[depth_img < self.max_detectable_distance] * self.depth_scaling_factor
        ground_loc = ground_loc * depth_val
        # print(np.shape(ground_loc), np.amin(depth_val), np.amax(depth_val))

        # compute raw_points
        raw_points = np.vstack([ground_loc, depth_val])

        # for efficiency, only convert max_points_to_convert points by taking random samples
        indices = np.random.choice(raw_points.shape[1], max_points_to_convert, replace=False)
        raw_points = raw_points[:, indices]

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
