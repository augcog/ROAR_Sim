from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.perception_module.ground_plane_point_cloud_detector import \
    GroundPlanePointCloudDetector
import numpy as np
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer
import open3d as o3d
from ROAR_simulation.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap
from ROAR_simulation.roar_autonomous_system.perception_module.gpd_detector import GroundPlaneDetector


class VisualizerDemoAgent(Agent):
    def __init__(self, **kwargs):
        super(VisualizerDemoAgent, self).__init__(**kwargs)
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
        self.gp_pointcloud_detector = GroundPlanePointCloudDetector(agent=self,
                                                                    max_points_to_convert=10000,
                                                                    nb_neighbors=100,
                                                                    std_ratio=1)
        self.occupancy_grid_map = OccupancyGridMap(absolute_maximum_map_size=800)
        self.semantic_segmentation_detector = GroundPlaneDetector(agent=self)
        self.visualizer = Visualizer(agent=self,
                                     occupancy_grid_map=self.occupancy_grid_map,
                                     semantic_segmentation_detector=self.semantic_segmentation_detector)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(VisualizerDemoAgent, self).run_step(sensors_data, vehicle)
        try:
            self.local_planner.run_step(vehicle=self.vehicle)

            # pcd: o3d.geometry.PointCloud = self.gp_pointcloud_detector.run_step()  # (N x 3)
            # self.occupancy_grid_map.update_grid_map_from_world_cord(np.asarray(pcd.points)[:, :2])
            # self.semantic_segmentation_detector.run_step()

            self.visualizer.show_first_person_visualization(show_num_waypoints=2,
                                                            show_semantic_segmentation_obstacle=False,
                                                            show_semantic_segmentation_sky=False,)
            # self.visualizer.show_birds_eye_visualization(focus_on_vehicle=True)

        except Exception as e:
            self.logger.error(f"Agent RunStep Error: {e}")
        finally:
            return self.local_planner.run_step(self.vehicle)
