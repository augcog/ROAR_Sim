from roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from roar_autonomous_system.control_module.pid_controller import VehiclePIDController
from roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from roar_autonomous_system.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from roar_autonomous_system.planning_module.mission_planner.waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from roar_autonomous_system.control_module.pid_controller import PIDParam
from roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from roar_autonomous_system.utilities_module.occupancy_map import OccupancyMap
from roar_autonomous_system.utilities_module.vehicle_models import VehicleControl, Vehicle
import logging
from roar_autonomous_system.utilities_module.data_structures_models import Transform
import numpy as np
import cv2


class WaypointFollowingAgent(Agent):
    def __init__(self, vehicle, route_file_path: Path, target_speed=40, **kwargs):
        super().__init__(vehicle, **kwargs)
        self.logger = logging.getLogger("PathFollowingAgent")
        self.route_file_path = route_file_path
        self.pid_controller = VehiclePIDController(vehicle=vehicle,
                                                   args_lateral=PIDParam.default_lateral_param(),
                                                   args_longitudinal=PIDParam.default_longitudinal_param(),
                                                   target_speed=target_speed)
        self.mission_planner = WaypointFollowingMissionPlanner(file_path=self.route_file_path, vehicle=vehicle)
        self.global_occupancy_grid_map = OccupancyMap.create_map(mission_plan=self.mission_planner.mission_plan,
                                                                 map_additional_padding=100)  # TODO revamp this
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(vehicle=vehicle,
                                                                 controller=self.pid_controller,
                                                                 mission_planner=self.mission_planner,
                                                                 behavior_planner=self.behavior_planner,
                                                                 closeness_threshold=1)

        self.logger.debug(f"Waypoint Following Agent Initiated. Reading from {route_file_path.as_posix()}")

    def run_step(self, vehicle: Vehicle, sensors_data: SensorsData) -> VehicleControl:
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)

        upcoming_waypoint:Transform = self.local_planner.way_points_queue[0]
        pos = self.calculate_img_pos(upcoming_waypoint, curr_veh_transform=vehicle.transform)
        img = sensors_data.front_rgb.data.copy()
        img[pos[1]:pos[1] + 10, pos[0]:pos[0] + 10] = [0, 0, 255]
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        return control

    def calculate_img_pos(self, waypoint_transform: Transform, curr_veh_transform: Transform):
        waypoint = np.array([waypoint_transform.location.x,
                             waypoint_transform.location.y,
                             waypoint_transform.location.z, 1])  # 4x1
        intrinsics = self.front_depth_camera.intrinsics_matrix # 3x3 matrix

        # 4x4 matrix
        cam_veh_matrix = self.calculate_extrinsics_from_euler(self.front_depth_camera.transform)
        veh_world_matrix = self.calculate_extrinsics_from_euler(curr_veh_transform)

        world_sensor_matrix = np.linalg.inv(cam_veh_matrix) @ np.linalg.inv(veh_world_matrix)
        cords_x_y_z = (world_sensor_matrix @ np.array(waypoint)).T

        cords_y_minus_z_x = np.array([cords_x_y_z[1], -cords_x_y_z[2], cords_x_y_z[0]])

        raw_p2d = (intrinsics @ cords_y_minus_z_x).T

        cam_coord = np.array([raw_p2d[0] / raw_p2d[2], raw_p2d[1] / raw_p2d[2], raw_p2d[2]])

        cam_coord = cam_coord.astype(np.int64)
        return cam_coord


    @staticmethod
    def calculate_extrinsics_from_euler(transform:Transform):
        location = transform.location
        rotation = transform.rotation
        yaw, pitch, roll = rotation.yaw, rotation.pitch, rotation.roll
        tx,ty, tz = location.x, location.y, location.z
        c_y = np.cos(np.radians(yaw))
        s_y = np.sin(np.radians(yaw))
        c_r = np.cos(np.radians(roll))
        s_r = np.sin(np.radians(roll))
        c_p = np.cos(np.radians(pitch))
        s_p = np.sin(np.radians(pitch))
        matrix = np.identity(4)
        matrix[0, 3] = tx
        matrix[1, 3] = ty
        matrix[2, 3] = tz
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix