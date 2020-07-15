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
from roar_autonomous_system.visualization_module.visualizer import Visualizer


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
        self.visualizer = Visualizer(agent=self)
        self.logger.debug(f"Waypoint Following Agent Initiated. Reading from {route_file_path.as_posix()}")
        self.min_error = 100
        self.max_error = 0

    def run_step(self, vehicle: Vehicle, sensors_data: SensorsData) -> VehicleControl:
        super(WaypointFollowingAgent, self).run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)

        try:

            from roar_autonomous_system.utilities_module.utilities import png_to_depth
            self.visualizer.visualize_waypoint(self.local_planner.way_points_queue[0])
            pos = self.visualizer.calculate_img_pos(waypoint_transform=self.local_planner.way_points_queue[0], camera=self.front_depth_camera)

            depth = png_to_depth(self.front_depth_camera.data)[pos[1]][pos[0]]
            depth = depth * 1000
            raw_p2d = np.array([pos[0]*depth, pos[1]*depth, depth])
            intrinsics = self.front_depth_camera.intrinsics_matrix
            cords_y_minus_z_x = np.linalg.inv(intrinsics) @ raw_p2d.T

            cords_x_y_z = np.array([cords_y_minus_z_x[2], cords_y_minus_z_x[0], -cords_y_minus_z_x[1], 1])

            cam_veh_matrix = self.front_depth_camera.get_matrix()
            veh_world_matrix = self.vehicle.get_matrix()
            world_sensor_matrix = np.linalg.inv(veh_world_matrix @ cam_veh_matrix)

            waypoint:np.array = np.linalg.inv(world_sensor_matrix) @ cords_x_y_z
            loc = self.local_planner.way_points_queue[0].location
            correct = np.array([loc.x, loc.y, loc.z, 1])
            diff = np.linalg.norm(correct - waypoint)
            self.min_error = diff if diff < self.min_error else self.min_error
            self.max_error = diff if diff > self.max_error else self.max_error

            from roar_autonomous_system.utilities_module.data_structures_models import Location
            waypoint_transform = Transform(location=Location(x=waypoint[0], y=waypoint[1], z=waypoint[2]))
            new_pos = self.visualizer.calculate_img_pos(waypoint_transform=waypoint_transform,
                                                        camera=self.front_depth_camera)

            print(f"Correct Coord: {np.round(correct, 3)} | my estimate = {np.round(waypoint,3)} | look ahead = {self.local_planner.closeness_threshold}")
            print(f"Abs Diff = {diff} | depth_from_depth_cam = {depth} | depth_from_waypoint_transformed_img = {pos[2]} | current speed: {Vehicle.get_speed(self.vehicle)}")
            print(f"pos_from_calculated_waypoint = {new_pos} | pos_from_true_waypoint = {pos}")
            print()

        except:
            pass
        return control
