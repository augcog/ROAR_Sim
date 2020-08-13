from ROAR_simulation.roar_autonomous_system.perception_module.detector import Detector
import logging
import open3d as o3d
import numpy as np
import cv2
import time
from typing import Optional
from ROAR_simulation.roar_autonomous_system.perception_module.point_cloud_detector import PointCloudDetector
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform, Location, Rotation


class GroundPlanePointCloudDetector(PointCloudDetector):
    def __init__(self,
                 max_ground_height_relative_to_vehcile=1,
                 knn=200,
                 std_ratio=2,
                 nb_neighbors=10,
                 **kwargs):
        """

        Args:
            max_detectable_distance: maximum detectable distance in km
            depth_scaling_factor: scaling depth back to world scale. 1000 m = 1 km
            **kwargs:
        """
        super().__init__(**kwargs)
        self.logger = logging.getLogger("Point Cloud Detector")

        self.max_ground_height_relative_to_vehcile = max_ground_height_relative_to_vehcile
        self.knn = knn
        self.std_ratio = std_ratio
        self.nb_neighbors = nb_neighbors
        self.counter = 0

    def run_step(self) -> o3d.geometry.PointCloud:
        points_3d = self.calculate_world_cords()  # (Nx3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)  # - np.mean(points_3d, axis=0))
        pcd.estimate_normals()
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)  # build KD tree for fast computation
        [k, idx, _] = pcd_tree.search_knn_vector_3d(self.agent.vehicle.transform.location.to_array(),
                                                    knn=self.knn)  # find points around me
        points_near_me = np.asarray(pcd.points)[idx, :]  # 200 x 3
        normals = np.asarray(pcd.normals)
        u, s, vh = np.linalg.svd(points_near_me, full_matrices=False)  # use svd to find normals of points
        avg_points_near_me_normal = vh[2, :]
        abs_diff = np.linalg.norm(normals - avg_points_near_me_normal, axis=1)  # anything below avg is plane
        planes = points_3d[abs_diff < np.mean(abs_diff)]

        ground = planes[planes[:, 2] < self.agent.vehicle.transform.location.z + 1]

        pcd.points = o3d.utility.Vector3dVector(ground)  # - np.mean(planes, axis=0))

        pcd, ids = pcd.remove_statistical_outlier(nb_neighbors=self.nb_neighbors, std_ratio=self.std_ratio)

        # self.pcd.points = pcd.points
        # if self.counter == 0:
        #     self.vis.create_window(window_name="Open3d", width=400, height=400)
        #     self.vis.add_geometry(self.pcd)
        #     render_option: o3d.visualization.RenderOption = self.vis.get_render_option()
        #     render_option.show_coordinate_frame = True
        # else:
        #     self.vis.update_geometry(self.pcd)
        #     render_option: o3d.visualization.RenderOption = self.vis.get_render_option()
        #     render_option.show_coordinate_frame = True
        #     self.vis.poll_events()
        #     self.vis.update_renderer()
        # self.counter += 1
        return pcd
