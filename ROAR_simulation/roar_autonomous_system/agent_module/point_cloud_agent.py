from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.perception_module.point_cloud_detector import GroundPlanePointCloudDetector
import numpy as np
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner\
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner\
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Location
from ROAR_simulation.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer


class PointCloudAgent(Agent):
    def __init__(self, **kwargs):
        super(PointCloudAgent, self).__init__(**kwargs)
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.controller = PurePursuitController(vehicle=self.vehicle, target_speed=20)
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=self.vehicle)
        self.behavior_planner = BehaviorPlanner(vehicle=self.vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=self.vehicle,
            controller=self.controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.gp_pointcloud_detector = GroundPlanePointCloudDetector(agent=self)
        self.visualizer = Visualizer(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(PointCloudAgent, self).run_step(sensors_data, vehicle)
        try:
            self.local_planner.run_step(vehicle=self.vehicle)
            points = self.gp_pointcloud_detector.run_step()
            print(np.amin(points, axis=0), np.amax(points, axis=0))
            # self.visualizer.visualize_waypoint(self.local_planner.way_points_queue[2])
            # next_waypoint_location: Location = self.local_planner.way_points_queue[2].location
            # points_on_the_same_y = points[np.abs(points[:, 1] - next_waypoint_location.y) < 2]
            # print(np.amin(points[:, 1], axis=0), np.amax(points[:, 1], axis=0), next_waypoint_location.y)
            # min_x = np.amin(points_on_the_same_y, axis=0)[0]
            # max_x = np.amax(points_on_the_same_y, axis=0)[0]
            # print(min_x, max_x)
            print()
        except Exception as e:
            self.logger.error(f"Point cloud RunStep Error: {e}")
        finally:
            return self.local_planner.run_step(self.vehicle)


