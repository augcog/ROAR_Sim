from roar_autonomous_system.agents.agent import Agent
from pathlib import Path
from roar_autonomous_system.control.pid_controller import VehiclePIDController
from roar_autonomous_system.planning.local_planners.simple_path_following_local_planner import \
    SimplePathFollowingLocalPlanner
from roar_autonomous_system.planning.behavior_planners.no_action_behavior_planner import NoActionBehaviorPlanner
from roar_autonomous_system.planning.mission_planners.path_following_mission_planner import PathFollowingMissionPlanner
from roar_autonomous_system.control.util import PIDParam
from roar_autonomous_system.util.models import Control, Vehicle, SensorData, OccupancyMap
from roar_autonomous_system.perception.ground_plane_detector import GroundPlaneDetector
from bridges.bridge import Bridge
from roar_autonomous_system.perception.utils import png_to_depth
import logging
import numpy as np
import cv2


class PathFollowingAgent(Agent):
    def __init__(self, vehicle, route_file_path: Path, bridge: Bridge, target_speed=200, visualize_occupancy_map=False):
        super().__init__(vehicle, bridge, visualize_occupancy_map)
        self.logger = logging.getLogger("PathFollowingAgent")
        self.route_file_path = route_file_path
        self.pid_controller = VehiclePIDController(vehicle=vehicle,
                                                   args_lateral=PIDParam.default_lateral_param(),
                                                   args_longitudinal=PIDParam.default_longitudinal_param(),
                                                   target_speed=target_speed)
        self.mission_planner = PathFollowingMissionPlanner(file_path=self.route_file_path)
        self.global_occupancy_grid_map = OccupancyMap.create_map(mission_plan=self.mission_planner.mission_plan,
                                                                 map_additional_padding=100)  # initiated right after mission plan
        self.behavior_planner = NoActionBehaviorPlanner(mission_path=self.mission_planner.mission_plan)
        self.local_planner = SimplePathFollowingLocalPlanner(vehicle=vehicle,
                                                             controller=self.pid_controller,
                                                             mission_planner=self.mission_planner,
                                                             behavior_planner=self.behavior_planner.generate_constraints(),
                                                             closeness_threshold=1)

        self.ground_plane_detector = GroundPlaneDetector(vehicle=self.vehicle)

        self.logger.debug(f"Path Following Agent Initiated. Reading from {route_file_path.as_posix()}")

    def run_step(self, vehicle: Vehicle, sensor_data: SensorData) -> Control:
        self.sync(vehicle=vehicle, sensor_data=sensor_data)
        self.transform_history.append(self.vehicle.transform)
        self.ground_plane_detector.run_step()
        if self.local_planner.is_done():
            control = Control()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            if self.global_occupancy_grid_map is not None and self.visualize_occupancy_map:
                self.global_occupancy_grid_map.visualize(
                    center_coord=self.global_occupancy_grid_map.convert_to_map_coord(self.vehicle.transform.location)[
                                 :2],
                    fov=300)
            control = self.local_planner.run_step()
        return control

    def sync(self, vehicle: Vehicle, sensor_data: SensorData):
        super(PathFollowingAgent, self).sync(vehicle=vehicle, sensor_data=sensor_data)
        self.local_planner.vehicle = self.vehicle  # on every run step, sync vehicle with lower level
        self.ground_plane_detector.curr_depth = sensor_data.front_depth
        self.ground_plane_detector.vehicle = vehicle

    @staticmethod
    def calculate_extrinsics_from_euler(pitch, yaw, roll, tx, ty, tz):
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