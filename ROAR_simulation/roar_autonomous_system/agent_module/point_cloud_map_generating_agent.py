from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner \
    .simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer import Visualizer
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController
from pydantic import BaseModel, Field
from typing import Tuple, List
from ROAR_simulation.roar_autonomous_system.perception_module.ground_plane_point_cloud_detector import \
    GroundPlanePointCloudDetector
import numpy as np
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Location


class MapEntry(BaseModel):
    point_a: List[float]
    point_b: List[float]


class PointCloudMapGeneratingAgent(Agent):
    def __init__(self, output_map_file_name: str = "generated_map.txt", **kwargs):
        super().__init__(**kwargs)
        self.output_map_file_path: Path = self.output_folder_path / output_map_file_name
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pid_controller = PurePursuitController(
            vehicle=self.vehicle,
            target_speed=40)
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=self.vehicle)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=self.vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=self.vehicle,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.visualizer = Visualizer(self)
        self.map_history: List[MapEntry] = []
        self.file_written = False

        self.point_cloud_ground_detector = GroundPlanePointCloudDetector(agent=self, nb_neighbors=100,
                                                                         max_ground_height_relative_to_vehcile=1)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(PointCloudMapGeneratingAgent, self).run_step(sensors_data, vehicle)
        control = self.local_planner.run_step(vehicle=self.vehicle)
        try:
            points: np.ndarray = self.point_cloud_ground_detector.run_step()
            # next_waypoint_location: Location = self.local_planner.way_points_queue[2].location
            # forward_vector = next_waypoint_location.to_array() - self.vehicle.transform.location.to_array()
            # forward_vector = forward_vector / np.linalg.norm(forward_vector)
            # norm_vector = np.linalg.svd(points, full_matrices=False)[2][:, 2]
            # left_pointing_vector = np.cross(forward_vector, norm_vector)
            # right_pointing_vector = np.cross(norm_vector, forward_vector)
            # print(left_pointing_vector, right_pointing_vector)
            min_x, min_y, _ = np.round(np.amin(points, axis=0), 0)
            max_x, max_y, _ = np.round(np.amax(points, axis=0), 0)
            curr_x, curr_y, _ = np.round(self.vehicle.transform.location.to_array(), 0)
            wayp_x, wayp_y, _ = np.round(self.local_planner.way_points_queue[2].location.to_array(), 0)
            print(min_x, min_y, "|", max_x, max_y, "|", curr_x, curr_y, "|", wayp_x, wayp_y)

            # print(, np.amax(points, axis=0), self.vehicle.transform.rotation)
        except Exception as e:
            self.logger.error(f"Error during map making: {e}")

        if self.local_planner.is_done() and self.file_written is False:
            self.logger.debug("WRITING TO FILE")
            f = self.output_map_file_path.open('w')
            import json
            json.dump(fp=f, obj=[map_entry.dict() for map_entry in self.map_history], indent=2)
            f.close()
            self.file_written = True

        return control
