from ROAR_simulation.roar_autonomous_system.perception_module.detector import Detector
import logging
import open3d as o3d
import numpy as np
import cv2
import time
from typing import Optional
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform, Location, Rotation


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

        self.counter = 0

    def run_step(self) -> Optional[np.ndarray]:
        points_3d = self.calculate_world_cords(max_points_to_convert=20000)  # (Nx3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d) #- np.mean(points_3d, axis=0))
        pcd.estimate_normals()

        # since i know that the closest 200 points are always ground, I will use them as a reference for the
        # normal vectors of the ground
        pcd_tree = o3d.geometry.KDTreeFlann(pcd) # build KD tree for fast computation
        [k, idx, _] = pcd_tree.search_knn_vector_3d(self.agent.vehicle.transform.location.to_array(), 200) # find points around me
        points_near_me = np.asarray(pcd.points)[idx, :]  # 200 x 3
        normals = np.asarray(pcd.normals)
        u, s, vh = np.linalg.svd(points_near_me, full_matrices=False)  # use svd to find normals of points
        avg_points_near_me_normal = vh[2, :]
        abs_diff = np.linalg.norm(normals - avg_points_near_me_normal, axis=1)  # anything below avg is plane
        planes = points_3d[abs_diff < np.mean(abs_diff)]

        # ground is anything below 1 m
        ground = planes[planes[:, 2] < self.agent.vehicle.transform.location.z + 1]
        pcd.points = o3d.utility.Vector3dVector(ground - np.mean(ground, axis=0))

        print(np.amin(ground, axis=0), np.amax(ground, axis=0), self.agent.vehicle.transform.location, np.shape(ground))

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
        return points_3d

    def calculate_world_cords(self, max_points_to_convert=5000, max_detectable_distance=0.05):
        depth_img = self.agent.front_depth_camera.data.copy()
        img_pos = np.indices((depth_img.shape[0], depth_img.shape[1])).transpose(1, 2, 0)
        img_pos = np.reshape(a=img_pos, newshape=(img_pos.shape[0] * img_pos.shape[1], 2))
        depth_array = np.reshape(a=depth_img, newshape=(depth_img.shape[0] * depth_img.shape[1], 1))

        indicies_to_delete = np.where(depth_array > max_detectable_distance)[0]
        depth_array = np.delete(depth_array, indicies_to_delete)
        img_pos = np.delete(img_pos, indicies_to_delete, axis=0)
        depth_array = depth_array * 1000

        indices_to_select = np.random.choice(np.shape(img_pos)[0],
                                             replace=False, size=min(np.shape(img_pos)[0], max_points_to_convert))

        p2d = np.append(img_pos, np.ones(shape=(np.shape(img_pos)[0], 1)), axis=1)

        p2d = np.take(p2d, indices_to_select, axis=0)
        depth_array = np.take(depth_array, indices_to_select, axis=0)
        # convert to raw_p2d
        raw_p2d = np.linalg.inv(self.agent.front_depth_camera.intrinsics_matrix) @ p2d.T

        # convert to cords_xyz_1
        cords_xyz_1 = np.vstack([
            raw_p2d[2, :] * depth_array.T,
            raw_p2d[1, :] * depth_array.T,
            -raw_p2d[0, :] * depth_array.T,
            np.ones((1, np.shape(raw_p2d)[1]))
        ])

        points = self.agent.vehicle.transform.get_matrix() @ self.agent.front_depth_camera.transform.get_matrix() \
                 @ cords_xyz_1
        return points[:3, :].T
