from ROAR.roar_autonomous_system.agent_module.agent import Agent
from ROAR.roar_autonomous_system.perception_module.depth_to_pointcloud_detector import DepthToPointCloudDetector
import numpy as np
from typing import Optional, Any
import open3d as o3d
import time, cv2


class GroundPlaneDetector(DepthToPointCloudDetector):
    def __init__(self, agent: Agent, knn: int = 200, *args):
        super().__init__(agent, *args)
        self.reference_norm: Optional[np.ndarray] = None
        self.knn = knn

    def run_step(self) -> Any:
        norm = np.array([-0.00994087, -0.99953604, 0.02879056], dtype=np.float32)
        # Setting up indices
        d1 = self.agent.front_depth_camera.image_size_y // 4
        d2 = self.agent.front_depth_camera.image_size_x // 4
        idx, jdx = np.indices((d1, d2))
        idx_back = np.clip(idx - 1, 0, idx.max()).flatten()
        idx_front = np.clip(idx + 1, 0, idx.max()).flatten()
        jdx_back = np.clip(jdx - 1, 0, jdx.max()).flatten()
        jdx_front = np.clip(jdx + 1, 0, jdx.max()).flatten()
        idx = idx.flatten()
        jdx = jdx.flatten()

        rand_idx = np.random.choice(np.arange(idx.shape[0]), size=d1 * d2, replace=False)
        f1 = (idx_front * d2 + jdx)[rand_idx]
        f2 = (idx_back * d2 + jdx)[rand_idx]
        f3 = (idx * d2 + jdx_front)[rand_idx]
        f4 = (idx * d2 + jdx_back)[rand_idx]
        norm_fill = np.zeros((idx.shape[0]))
        t1 = time.time()
        points = super(GroundPlaneDetector, self).run_step()
        t2 = time.time()
        time_check1 = t2 - t1

        t3 = time.time()

        x = points[f1, :] - points[f2, :]
        y = points[f3, :] - points[f4, :]

        xyz_norm = self.normalize_v3(np.cross(x, y))
        if self.reference_norm is None:
            pcd = self.contruct_pointcloud(points)
            # pointcloud_normals = np.asarray(pcd.normals)

            if self.reference_norm is None:
                self.compute_reference_norm(pcd=pcd)

        norm_flat = xyz_norm @ self.reference_norm

        norm_fill[rand_idx] = norm_flat
        norm_matrix = np.abs(norm_fill.reshape((d1, d2)))
        print(np.amin(norm_fill), np.amax(norm_fill))

        norm_umatrix = cv2.resize(cv2.UMat(norm_matrix), (d2 * 4, d1 * 4))
        bool_matrix = cv2.compare(norm_umatrix, 0.4, cmpop=cv2.CMP_GT)
        comps, out, stats, cents = cv2.connectedComponentsWithStats(bool_matrix, connectivity=4)
        sizes = stats.get()[:, 2]  # get the area sizes
        max_label = np.argmax(sizes[2:comps]) + 2
        color_image = self.agent.front_rgb_camera.data.copy()
        color_image[out.get() == max_label] = 255
        t2 = time.time()

        text = f'FPS: {1 / (t2 - t1)}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 0, 0)
        place = (50, 50)
        thicc = 2

        color_image = cv2.putText(
            color_image, text, place, font, 1, color, thicc, cv2.LINE_AA
        )
        cv2.imshow('Color', color_image)
        # pcd = self.contruct_pointcloud(points)
        # pointcloud_normals = np.asarray(pcd.normals)
        #
        # if self.reference_norm is None:
        #     self.compute_reference_norm(pcd=pcd)
        #
        # norm_flat = pointcloud_normals @ self.reference_norm
        # norm_matrix = np.abs(norm_flat.reshape((d1, d2)))
        # norm_umatrix = cv2.resize(cv2.UMat(norm_matrix), (d1, d2))
        # bool_matrix = cv2.compare(norm_umatrix, 0.95, cmpop=cv2.CMP_GT)
        # comps, out, stats, cents = cv2.connectedComponentsWithStats(bool_matrix, connectivity=4)
        # sizes = stats.get()[:, 2]  # get the area sizes
        # max_label = np.argmax(sizes[2:comps]) + 2
        # color_image = self.agent.front_rgb_camera.data.copy()
        # color_image[out.get() == max_label] = 255
        # cv2.imshow('Color', color_image)
        # cv2.waitKey(1)
        # t4 = time.time()
        # time_check2 = t4 - t3
        # # print(time_check1, time_check2, 1 / time_check1, 1 / time_check2)

    @staticmethod
    def contruct_pointcloud(points) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.estimate_normals()
        return pcd

    def compute_reference_norm(self, pcd: o3d.geometry.PointCloud):
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)  # build KD tree for fast computation
        [k, idx, _] = pcd_tree.search_knn_vector_3d(self.agent.vehicle.transform.location.to_array(),
                                                    knn=self.knn)  # find points around me
        points_near_me = np.asarray(pcd.points)[idx, :]  # 200 x 3
        u, s, vh = np.linalg.svd(points_near_me, full_matrices=False)  # use svd to find normals of points
        self.reference_norm = vh[2, :]

    @staticmethod
    def normalize_v3(arr):
        lens = np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2 + arr[:, 2] ** 2)
        lens[lens <= 0] = 1
        arr[:, 0] /= lens
        arr[:, 1] /= lens
        arr[:, 2] /= lens
        return arr