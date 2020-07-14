import logging
from roar_autonomous_system.utilities_module.data_structures_models import Transform
from roar_autonomous_system.utilities_module.utilities import calculate_extrinsics_from_euler
import numpy as np
import cv2


class Visualizer:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)
        self.agent = agent

    def visualize_waypoint(self, waypoint_transform: Transform):
        coord = self.calculate_img_pos(waypoint_transform=waypoint_transform,
                                       curr_veh_transform=self.agent.vehicle.transform,
                                       curr_cam_transform=self.agent.front_depth_camera.transform,
                                       cam_intrinsics=self.agent.front_depth_camera.intrinsics_matrix)
        img = self.agent.front_rgb_camera.data.copy()
        start_point = (400, 600)
        end_point = (coord[0], coord[1])
        color = (0, 255, 0)
        thickness = 2
        img = cv2.arrowedLine(img, start_point, end_point,
                              color, thickness)
        # img[coord[1]:coord[1]+5,coord[0]:coord[0]+5] = [0,0,255]
        cv2.imshow("Next Waypoint", img)
        cv2.waitKey(1)

    def calculate_img_pos(self,
                          waypoint_transform: Transform,
                          curr_veh_transform: Transform,
                          curr_cam_transform: Transform,
                          cam_intrinsics: np.array):
        """
        Calculate the 2D image coordinate from 3D world space

        Args:
            waypoint_transform: Desired point in 3D world space
            curr_veh_transform: current vehicle transform with respect to world,
            curr_cam_transform: current camera transform with respect to vehicle
            cam_intrinsics: current camera intrinsics

        Returns:
            Array if integers [X, Y, ANYNUM]

        """
        waypoint = np.array([waypoint_transform.location.x,
                             waypoint_transform.location.y,
                             waypoint_transform.location.z, 1])  # 4x1
        intrinsics = cam_intrinsics  # 3x3 matrix

        # 4x4 matrix
        cam_veh_matrix = calculate_extrinsics_from_euler(curr_cam_transform)
        veh_world_matrix = calculate_extrinsics_from_euler(curr_veh_transform)

        world_sensor_matrix = np.linalg.inv(cam_veh_matrix) @ np.linalg.inv(veh_world_matrix)
        cords_x_y_z = (world_sensor_matrix @ np.array(waypoint)).T
        print("visualizer", cords_x_y_z)
        cords_y_minus_z_x = np.array([cords_x_y_z[1], -cords_x_y_z[2], cords_x_y_z[0]])

        raw_p2d = (intrinsics @ cords_y_minus_z_x).T

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
                                                       curr_veh_transform=self.agent.vehicle.transform,
                                                       curr_cam_transform=self.agent.front_depth_camera.transform,
                                                       cam_intrinsics=self.agent.front_depth_camera.intrinsics_matrix)
        img = self.agent.front_rgb_camera.data.copy()

        start_point = (400, 600)

        img = cv2.arrowedLine(img=img,
                              pt1=start_point,
                              pt2=(next_waypoint_cam_pos[0], next_waypoint_cam_pos[1]),
                              color=(0, 255, 0),
                              thickness=2)
        cv2.imshow("Visualization", img)
        cv2.waitKey(1)


