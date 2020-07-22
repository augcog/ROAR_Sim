from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import (
    VehiclePIDController,
)
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner \
    .simple_waypoint_following_local_planner import (
    SimpleWaypointFollowingLocalPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import (
    BehaviorPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import (
    WaypointFollowingMissionPlanner,
)
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import (
    PIDParam,
)
from ROAR_simulation.roar_autonomous_system.utilities_module \
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
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import \
    png_to_depth, img_to_world
from ROAR_simulation.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap

import cv2
import numpy as np


class WaypointFollowingAgent(Agent):
    def __init__(self, vehicle, route_file_path: Path, target_speed=40, *args,
                 **kwargs):
        super().__init__(vehicle, *args, **kwargs)
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
        self.occupancy_grid_map = OccupancyGridMap(min_x=-500,
                                                   min_y=-500,
                                                   max_x=500,
                                                   max_y=500)
        self.visualizer = Visualizer(agent=self)

        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {route_file_path.as_posix()}"
        )



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

            try:
                veh_cam_matrix = \
                    self.front_depth_camera.transform.get_matrix()  # 4 x 4
                world_veh_matrix = self.vehicle.transform.get_matrix()
                cam_world_matrix = world_veh_matrix @ veh_cam_matrix

                depth_img = self.front_depth_camera.data.copy()
                # print(np.shape(depth_img))
                world_cords = img_to_world(
                    depth_img=depth_img,
                    intrinsics_matrix=self.front_depth_camera.intrinsics_matrix,
                    extrinsics_matrix=cam_world_matrix)
                self.occupancy_grid_map.update_grid_map(world_cords_xy=world_cords[:2, :].T)
                self.occupancy_grid_map.visualize(vehicle_location=self.vehicle.transform.location)
            except Exception as e:
                print(f"Failed: {e}")
        return control
