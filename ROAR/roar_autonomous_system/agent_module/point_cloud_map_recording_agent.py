from ROAR.roar_autonomous_system.agent_module.agent import Agent
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.roar_autonomous_system.perception_module.ground_plane_point_cloud_detector import GroundPlanePointCloudDetector
from ROAR.roar_autonomous_system.visualization_module.visualizer import Visualizer
import numpy as np
import cv2
from pathlib import Path
from ROAR.roar_autonomous_system.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.roar_autonomous_system.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import SimpleWaypointFollowingLocalPlanner
from ROAR.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController

class PointCloudMapRecordingAgent(Agent):
    def __init__(self, **kwargs):
        super(PointCloudMapRecordingAgent, self).__init__(**kwargs)
        self.logger.debug("GPD2 Agent Initialized")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        print(self.route_file_path)
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=self.vehicle)
        # initiated right after mission plan
        self.controller = \
            PurePursuitController(vehicle=self.vehicle,
                                  target_speed=60,
                                  look_ahead_gain=0.1,
                                  look_ahead_distance=3)
        self.behavior_planner = BehaviorPlanner(vehicle=self.vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=self.vehicle,
            controller=self.controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.ground_plane_point_cloud_detector = GroundPlanePointCloudDetector(agent=self, max_points_to_convert=20000)
        self.visualizer = Visualizer(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(PointCloudMapRecordingAgent, self).run_step(sensors_data=sensors_data, vehicle=vehicle)
        ground_points = self.ground_plane_point_cloud_detector.run_step()

        color_image = self.front_rgb_camera.data.copy()
        image_coords: np.ndarray = self.visualizer.world_to_img_transform(xyz=ground_points)[:,:2]
        # color_image[image_coords[:, 1], image_coords[:, 0]] = [255, 255, 255]
        # self.visualizer.show_first_person_visualization(show_num_waypoints=1)
        img_pos = self.visualizer.world_to_img_transform(np.array([self.local_planner.way_points_queue[0].location.to_array()]))
        for y, x, _ in img_pos:
            print(x,y, self.local_planner.way_points_queue[0].location)
            color_image[x-2: x+2, y-2:y+2] = self.visualizer.GREEN
        cv2.imshow("color", color_image)
        cv2.waitKey(1)
        return VehicleControl()
