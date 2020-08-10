from ROAR_simulation.roar_autonomous_system.perception_module.detector import Detector
import logging
import open3d as o3d
import numpy as np
import cv2
import time
from typing import Optional
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform, Location, Rotation


class GroundPlanePointCloudDetector(Detector):
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

        self.counter = 0

    def run_step(self) -> Optional[np.ndarray]:
        points_3d = self.calculate_world_cords(max_points_to_convert=10000)  # (Nx3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)# - np.mean(points_3d, axis=0))
        pcd.estimate_normals()
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)  # build KD tree for fast computation
        [k, idx, _] = pcd_tree.search_knn_vector_3d(self.agent.vehicle.transform.location.to_array(),
                                                    200)  # find points around me
        points_near_me = np.asarray(pcd.points)[idx, :]  # 200 x 3
        normals = np.asarray(pcd.normals)
        u, s, vh = np.linalg.svd(points_near_me, full_matrices=False)  # use svd to find normals of points
        avg_points_near_me_normal = vh[2, :]
        abs_diff = np.linalg.norm(normals - avg_points_near_me_normal, axis=1)  # anything below avg is plane
        planes = points_3d[abs_diff < np.mean(abs_diff)]

        ground = planes[planes[:, 2] < self.agent.vehicle.transform.location.z + 3]
        # print(np.amin(ground, axis=0), np.amax(ground, axis=0), self.agent.vehicle.transform.location, np.shape(ground))
        pcd.points = o3d.utility.Vector3dVector(ground)  # - np.mean(planes, axis=0))

        pcd, ids = pcd.remove_statistical_outlier(10, 2)

        self.pcd.points = pcd.points
        if self.counter == 0:
            self.vis.create_window(window_name="Open3d", width=400, height=400)
            self.vis.add_geometry(self.pcd)
            render_option: o3d.visualization.RenderOption = self.vis.get_render_option()
            render_option.show_coordinate_frame = True
        else:
            self.vis.update_geometry(self.pcd)
            render_option: o3d.visualization.RenderOption = self.vis.get_render_option()
            render_option.show_coordinate_frame = True
            self.vis.poll_events()
            self.vis.update_renderer()
        self.counter += 1
        return np.asarray(self.pcd.points)

    def calculate_world_cords(self, max_points_to_convert=5000, max_detectable_distance=0.05):
        depth_img = self.agent.front_depth_camera.data.copy()
        coords = np.where(depth_img < max_detectable_distance)

        indices_to_select = np.random.choice(np.shape(coords)[1], size=max_points_to_convert, replace=False)

        coords = (
            coords[0][indices_to_select],
            coords[1][indices_to_select]
        )

        raw_p2d = []
        for i in range(np.shape(coords)[1]):
            x, y = coords[0][i], coords[1][i]
            depth = depth_img[x][y] * 1000
            raw_p2d.append(
                [y * depth, x * depth, depth]
            )
        raw_p2d = np.array(raw_p2d)

        cords_y_minus_z_x = np.linalg.inv(self.agent.front_depth_camera.intrinsics_matrix) @ raw_p2d.T
        cords_xyz_1 = np.vstack([
            cords_y_minus_z_x[2, :],
            cords_y_minus_z_x[0, :],
            -cords_y_minus_z_x[1, :],
            np.ones((1, np.shape(cords_y_minus_z_x)[1]))
        ])
        points: np.ndarray = self.agent.vehicle.transform.get_matrix() @ self.agent.front_depth_camera.transform.get_matrix() @ cords_xyz_1
        points = points.T[:, :3]

        # print(np.shape(points), np.amin(points, axis=0), np.amax(points, axis=0))
        return points
