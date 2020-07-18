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
from roar_autonomous_system.utilities_module.utilities import png_to_depth


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
        self.curr_max_err = 0
        self.counter = 0
        self.total_err = 0

    def run_step(self, vehicle: Vehicle, sensors_data: SensorsData) -> VehicleControl:
        super(WaypointFollowingAgent, self).run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)
            try:
                # self.visualizer.visualize_waypoint(self.local_planner.way_points_queue[0])
                waypoint_location_0 = self.local_planner.way_points_queue[0]
                waypoint_location_1 = self.local_planner.way_points_queue[1]
                waypoint_location_2 = self.local_planner.way_points_queue[2]

                pos0 = self.visualizer.calculate_img_pos(waypoint_location_0, self.front_depth_camera)
                pos1 = self.visualizer.calculate_img_pos(waypoint_location_1, self.front_depth_camera)
                pos2 = self.visualizer.calculate_img_pos(waypoint_location_2, self.front_depth_camera)

                print(pos0, pos1, pos2)

                curr_image = self.front_rgb_camera.data.copy()
                curr_image[pos0[1]: pos0[1] + 5, pos0[0]: pos0[0] + 5] = [0, 255, 0]
                curr_image[pos1[1]: pos1[1] + 5, pos1[0]: pos1[0] + 5] = [255, 255, 0]
                curr_image[pos2[1]: pos2[1] + 5, pos2[0]: pos2[0] + 5] = [0, 0, 255]



                cv2.imshow("image", curr_image)
                cv2.waitKey(1)

                # projected_waypoint_0 = self.img_to_world(pos0)
                # print(f"Actual_Waypoint_0 = {np.round(waypoint_location_0.location.to_array(), 3)} "
                #       f"| projected waypoint 0 = {np.round(projected_waypoint_0, 3)} "
                #       f"| abs_diff = {np.linalg.norm(waypoint_location_0.location.to_array()[:2] - projected_waypoint_0[:2])}")
                # print()
            except:
                print("Failed")
                pass
        return control

    def img_to_world(self, img_coords: np.array):
        depth = png_to_depth(self.front_depth_camera.data)[img_coords[1]][img_coords[0]] * 1000

        P = np.linalg.inv(self.front_depth_camera.intrinsics_matrix) @ np.array([img_coords[0], img_coords[1], 1]) * depth

        cords_xyz = np.array([P[2], P[0], -P[1], 1])
        veh_cam_matrix = self.front_depth_camera.transform.get_matrix()  # 4 x 4
        world_veh_matrix = self.vehicle.transform.get_matrix()  # 4 x 4

        cam_world = np.dot(world_veh_matrix, veh_cam_matrix)

        waypoint_location = cam_world @ cords_xyz

        return waypoint_location

