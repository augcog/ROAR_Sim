from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.perception_module.gpd_detector import GroundPlaneDetector
import cv2
import numpy as np
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import img_to_world

class MapGeneratingAgent(Agent):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.output_map_file_path = self.output_folder_path / "generated_map.txt"
        self.ground_plane_detector = GroundPlaneDetector(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(MapGeneratingAgent, self).run_step(sensors_data, vehicle)
        self.ground_plane_detector.run_step()

        if self.ground_plane_detector.curr_segmentation is not None and \
                len(self.ground_plane_detector.curr_segmentation) > 0:

            seg_visual = self.ground_plane_detector.curr_segmentation.copy()

            # define center point
            center = np.array([370, 400])
            seg_visual[center[0]-5:center[0] + 5, center[1]-5:center[1] + 5] = [0,255,0]

            # find edge points
            # find left
            left_obstacle_pos = np.array([center[0], 0])
            for x in range(center[1], 0, -1):
                if np.any(self.ground_plane_detector.curr_segmentation[center[0], x] == self.ground_plane_detector.OBSTACLE):
                    left_obstacle_pos[1] = x
                    break
            # find right
            right_obstacle_pos = np.array([center[0], np.shape(seg_visual)[1]-1])
            for x in range(center[1], np.shape(seg_visual)[1], 1):
                if np.any(self.ground_plane_detector.curr_segmentation[center[0], x] == self.ground_plane_detector.OBSTACLE):
                    right_obstacle_pos[1] = x
                    break

            # find corresponding depth values
            depth_left = self.front_depth_camera.data[left_obstacle_pos[0], left_obstacle_pos[1]]
            depth_middle = self.front_depth_camera.data[center[0], center[1]]
            depth_right = self.front_depth_camera.data[right_obstacle_pos[0], right_obstacle_pos[1]]

            depth_data = self.front_depth_camera.data.copy()

            depth_data[left_obstacle_pos[0]:left_obstacle_pos[0]+5, left_obstacle_pos[1]:left_obstacle_pos[1]+5] = 1
            depth_data[center[0]:center[0] + 5, center[1]:center[1] + 5] = 1
            depth_data[right_obstacle_pos[0]:right_obstacle_pos[0] + 5, right_obstacle_pos[1]:right_obstacle_pos[1] + 5] = 1
            cv2.imshow("depth", depth_data)
            cv2.waitKey(1)


            # transform into world cords
            ground_loc = np.array([left_obstacle_pos, center, right_obstacle_pos]).T

            depth_val = np.array([depth_left, depth_middle, depth_right]) * 1000
            ground_loc = ground_loc * depth_val

            # compute raw_points
            raw_points = np.vstack([ground_loc, depth_val])
            # print(raw_points)

            # convert to cords_y_minus_z_x
            cords_y_minus_z_x = np.linalg.inv(self.front_depth_camera.intrinsics_matrix) @ raw_points

            # convert to cords_xyz_1
            ones = np.ones((1, np.shape(cords_y_minus_z_x)[1]))

            cords_xyz_1 = np.vstack([
                cords_y_minus_z_x[2, :],
                cords_y_minus_z_x[0, :],
                -cords_y_minus_z_x[1, :],
                ones
            ])

            # multiply by cam_world_matrix
            points = self.front_depth_camera.transform.get_matrix() @ self.vehicle.transform.get_matrix() @ cords_xyz_1  # i have all points now

            points = points[:2].T
            print(f"Vehicle Current Position = {self.vehicle.transform.location}")
            print(f"Obstacle Position = \n{points}")
            # save to txt

            cv2.imshow("curr_segmentation", seg_visual)
            cv2.waitKey(1)

        return VehicleControl()