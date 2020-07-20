from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import (
    VehiclePIDController,
)
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner\
    .simple_waypoint_following_local_planner import (
    SimpleWaypointFollowingLocalPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner\
    .behavior_planner import (
    BehaviorPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner\
    .waypoint_following_mission_planner import (
    WaypointFollowingMissionPlanner,
)
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import (
    PIDParam,
)
from ROAR_simulation.roar_autonomous_system.utilities_module\
    .data_structures_models import (
    SensorsData,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import (
    VehicleControl,
    Vehicle,
)
import logging
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import (
    Visualizer,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import png_to_depth

import cv2
import numpy as np


class WaypointFollowingAgent(Agent):
    def __init__(self, vehicle, route_file_path: Path, target_speed=40,
                 **kwargs):
        super().__init__(vehicle, **kwargs)
        self.logger = logging.getLogger("PathFollowingAgent")
        self.route_file_path = route_file_path
        self.pid_controller = VehiclePIDController(
            vehicle=vehicle,
            args_lateral=PIDParam.default_lateral_param(),
            args_longitudinal=PIDParam.default_longitudinal_param(),
            target_speed=target_speed,
        )
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=vehicle
        )
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=vehicle,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1,
        )
        self.visualizer = Visualizer(agent=self)
        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {route_file_path.as_posix()}"
        )
        self.curr_max_err = 0
        self.counter = 0
        self.total_err = 0

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(WaypointFollowingAgent, self).run_step(
            vehicle=vehicle, sensors_data=sensors_data
        )
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)

            waypoint0 = self.local_planner.way_points_queue[0]
            waypoint1 = self.local_planner.way_points_queue[1]
            waypoint2 = self.local_planner.way_points_queue[2]


            pos0 = self.visualizer.calculate_img_pos(
                waypoint_transform=waypoint0,
                camera=self.front_depth_camera)
            pos1 = self.visualizer.calculate_img_pos(
                waypoint_transform=waypoint1,
                camera=self.front_depth_camera)
            pos2 = self.visualizer.calculate_img_pos(
                waypoint_transform=waypoint2,
                camera=self.front_depth_camera)

            image = self.front_rgb_camera.data.copy()

            image[pos0[1]:pos0[1] + 5, pos0[0]:pos0[0] + 5] = [0, 255, 255]
            # image[pos1[1]:pos1[1] + 5, pos1[0]:pos1[0] + 5] = [255, 0, 0]
            # image[pos2[1]:pos2[1] + 5, pos2[0]:pos2[0] + 5] = [255, 0, 255]

            cv2.imshow("image", image)
            cv2.waitKey(1)
            # print("************")
            # print(f"depth0 0 1 | {self.front_depth_camera.data[pos0[0]][pos0[1]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos0[0]][pos0[1]] * 1000}")
            # print(
            #     f"depth0 1 0 | {self.front_depth_camera.data[pos0[1]][pos0[0]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos0[1]][pos0[0]] * 1000}")
            # print()
            # print(f"depth1 0 1 | {self.front_depth_camera.data[pos1[0]][pos1[1]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos1[0]][pos1[1]] * 1000}")
            # print(
            #     f"depth1 1 0 | {self.front_depth_camera.data[pos1[1]][pos1[0]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos1[1]][pos1[0]] * 1000}")
            # print()
            # print(f"depth2 0 1 | {self.front_depth_camera.data[pos2[0]][pos2[1]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos2[0]][pos2[1]] * 1000}")
            # print(
            #     f"depth2 1 0 | {self.front_depth_camera.data[pos2[1]][pos2[0]]} -> depth = {png_to_depth(self.front_depth_camera.data)[pos2[1]][pos2[0]] * 1000}")
            #
            # array = self.front_depth_camera.data[pos0[1]][pos0[0]]
            # result = (array[0] + array[1] * 256 + array[2] * 256 * 256) / (256 * 256 * 256 - 1)
            # print(result)
            # print("************")
            # print()


            depth0 = png_to_depth(self.front_depth_camera.data)[pos0[1]][pos0[0]] * 1000
            # depth1 = png_to_depth(self.front_depth_camera.data)[pos1[1]][pos1[0]] * 1000

            if self.counter % 5 == 0:
                print(f"depth diff = {abs(pos0[2] - depth0)} | depth = {depth0} "
                      f"| transformed_depth = {pos0[2]} | norm = {np.round(np.linalg.norm(self.vehicle.transform.location.to_array() - waypoint0.location.to_array()), 3)}")
            self.counter += 1
            depth_array = png_to_depth(self.front_depth_camera.data)
            depth_array[depth_array > 0.089] = 0
            depth_array = depth_array / np.amax(depth_array)
            cv2.imshow("depthimg", depth_array)
            cv2.waitKey(1)
            # print(f"depth = {depth1} | transformed_depth = {pos1[2]} |
            # curr_veh= {np.round(self.vehicle.transform.location.to_array(
            # ), 3)}" f" | acutal = {np.round(waypoint1.location.to_array(),
            # 3)}") print() raw_p2d = np.array([pos0[0] * depth0, pos0[1] *
            # depth0, 1]) cords_y_minus_z_x = np.linalg.inv(
            # self.front_depth_camera.intrinsics_matrix) @ raw_p2d cords_xyz
            # = np.array([cords_y_minus_z_x[2], cords_y_minus_z_x[0],
            # -cords_y_minus_z_x[1]])
            #
            # veh_cam_matrix = self.front_depth_camera.transform.get_matrix()  # 4 x 4
            # world_veh_matrix = self.vehicle.transform.get_matrix()  # 4 x 4
            #
            # world_cam_matrix = np.linalg.inv(
            #     np.dot(world_veh_matrix, veh_cam_matrix))
            #
            # waypoint_location = np.linalg.inv(world_cam_matrix) @ cords_xyz
            #
            # print(f"Correct Waypoint = {np.round(waypoint0.location.to_array() ,3)}"
            #       f" | Projected Waypoint = {np.round(waypoint_location, 3)} "
            #       f"| abs error {np.linalg.norm( waypoint0.location.to_array() - waypoint_location )}")



        return control
