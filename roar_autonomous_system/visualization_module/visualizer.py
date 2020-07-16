import logging
from roar_autonomous_system.utilities_module.data_structures_models import Transform
from roar_autonomous_system.utilities_module.camera_models import Camera
import numpy as np
import cv2
from roar_autonomous_system.agent_module.agent import Agent


class Visualizer:
    def __init__(self, agent: Agent):
        self.logger = logging.getLogger(__name__)
        self.agent = agent

    def visualize_waypoint(self, waypoint_transform: Transform):
        coord = self.calculate_img_pos(waypoint_transform=waypoint_transform,
                                       camera=self.agent.front_depth_camera)
        img = self.agent.front_rgb_camera.data.copy()
        start_point = (400, 600)
        end_point = (coord[0], coord[1])
        color = (0, 255, 0)
        thickness = 2
        img = cv2.arrowedLine(img, start_point, end_point,
                              color, thickness)
        cv2.imshow("Next Waypoint", img)
        cv2.waitKey(1)

    def calculate_img_pos(self,
                          waypoint_transform: Transform, camera: Camera):
        """
        Calculate the 2D image coordinate from 3D world space

        Args:
            camera:
            waypoint_transform: Desired point in 3D world space

        Returns:
            Array if integers [X, Y, depth]

        """
        waypoint = np.array([waypoint_transform.location.x,
                             waypoint_transform.location.y,
                             waypoint_transform.location.z, 1])  # 4x1
        intrinsics = camera.intrinsics_matrix  # 3x3 matrix

        # 4x4 matrix
        cam_veh_matrix = camera.get_matrix()
        veh_world_matrix = self.agent.vehicle.get_matrix()

        sensor_world_matrix_2 = cam_veh_matrix @ veh_world_matrix
        tmp = np.dot(np.linalg.inv(sensor_world_matrix_2), waypoint)
        tmp = np.array([tmp[1], -tmp[2], tmp[0]])
        pos2d = np.dot(self.agent.front_depth_camera.intrinsics_matrix, tmp[:3])
        pos2d = np.array([
            pos2d[0] / pos2d[2],
            pos2d[1] / pos2d[2],
            pos2d[2]
        ])
        pos2d = pos2d.astype(np.int64)
        print(pos2d)

        # world_sensor_matrix = np.dot(veh_world_matrix, cam_veh_matrix)
        # tmp = np.dot(np.linalg.inv(world_sensor_matrix), waypoint)
        # tmp = np.array([tmp[1], -tmp[2], tmp[0]])
        # pos2d = np.dot(intrinsics, tmp[:3])
        # pos2d = np.array([
        #     pos2d[0] / pos2d[2],
        #     pos2d[1] / pos2d[2],
        #     pos2d[2]
        # ])
        # pos2d = pos2d.astype(np.int64)
        # # return pos2d
        # # print(pos2d)

        world_sensor_matrix = np.linalg.inv(np.matmul(cam_veh_matrix, veh_world_matrix))

        cords_x_y_z = np.matmul(world_sensor_matrix, waypoint)

        cords_y_minus_z_x = np.array([cords_x_y_z[1], -cords_x_y_z[2], cords_x_y_z[0]])
        raw_p2d = np.matmul(intrinsics, cords_y_minus_z_x)
        cam_coord = np.array([raw_p2d[0] / raw_p2d[2], raw_p2d[1] / raw_p2d[2], raw_p2d[2]])

        cam_coord = cam_coord.astype(np.int64)
        return cam_coord

    def visualize(self, next_waypoint_transform: Transform):
        """
        This function will allow multiple objects to be drawn on here.

        Currently implemented are
        1. Next Waypoint


        Args:
            next_waypoint_transform: location & rotation of the next waypoint

        Returns:

        """
        next_waypoint_cam_pos = self.calculate_img_pos(waypoint_transform=next_waypoint_transform,
                                                       camera=self.agent.front_depth_camera)
        img = self.agent.front_rgb_camera.data.copy()

        start_point = (400, 600)

        img = cv2.arrowedLine(img=img,
                              pt1=start_point,
                              pt2=(next_waypoint_cam_pos[0], next_waypoint_cam_pos[1]),
                              color=(0, 255, 0),
                              thickness=2)
        cv2.imshow("Visualization", img)
        cv2.waitKey(1)

    @classmethod
    def visualize_semantic_segmentation(cls, semantic_segmetation):
        """

        Args:
            semantic_segmetation: Width x Height x 3 array with white = obstacles, black = ground, blue = sky

        Returns:

        """

        if semantic_segmetation is not None:
            cv2.imshow("Semantic Segmentation", semantic_segmetation)
            cv2.waitKey(1)


