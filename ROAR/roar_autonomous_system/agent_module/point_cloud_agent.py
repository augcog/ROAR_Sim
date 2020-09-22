from ROAR.roar_autonomous_system.agent_module.agent import Agent
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.roar_autonomous_system.perception_module.ground_plane_point_cloud_detector import \
    GroundPlanePointCloudDetector
import numpy as np
from ROAR.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import \
    BehaviorPlanner
from ROAR.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController
from pathlib import Path
from ROAR.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer
import open3d as o3d
from ROAR.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap


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
        self.gp_pointcloud_detector = GroundPlanePointCloudDetector(agent=self,
                                                                    max_points_to_convert=10000,
                                                                    nb_neighbors=100,
                                                                    std_ratio=1)
        self.occupancy_grid_map = OccupancyGridMap(absolute_maximum_map_size=800)
        self.visualizer = Visualizer(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(PointCloudAgent, self).run_step(sensors_data, vehicle)
        try:
            self.local_planner.run_step(vehicle=self.vehicle)
            points = self.gp_pointcloud_detector.run_step()  # (N x 3)
            print(np.amin(points, axis=0), np.amax(points, axis=0), self.vehicle.transform.location.to_array())
            # self.occupancy_grid_map.update_grid_map_from_world_cord(points[:, :2])
            # self.occupancy_grid_map.visualize(vehicle_location=self.vehicle.transform.location)

        except Exception as e:
            self.logger.error(f"Point cloud RunStep Error: {e}")
        finally:
            return self.local_planner.run_step(self.vehicle)
    def calculate_world_cords(self):
        depth_img = self.agent.front_depth_camera.data.copy()

        coords = np.where(depth_img < self.max_detectable_distance)

        indices_to_select = np.random.choice(np.shape(coords)[1],
                                             size=min([self.max_points_to_convert, np.shape(coords)[1]]),
                                             replace=False)

        coords = (
            coords[0][indices_to_select],
            coords[1][indices_to_select]
        )

        raw_p2d = np.reshape(self._pix2xyz(depth_img=depth_img, i=coords[0], j=coords[1]), (3, np.shape(coords)[1])).T

        cords_y_minus_z_x = np.linalg.inv(self.agent.front_depth_camera.intrinsics_matrix) @ raw_p2d.T
        cords_xyz_1 = np.vstack([
            cords_y_minus_z_x[2, :],
            cords_y_minus_z_x[0, :],
            -cords_y_minus_z_x[1, :],
            np.ones((1, np.shape(cords_y_minus_z_x)[1]))
        ])
        points: np.ndarray = self.agent.vehicle.transform.get_matrix() @ self.agent.front_depth_camera.transform.get_matrix() @ cords_xyz_1
        points = points.T[:, :3]
        return points

    @staticmethod
    def _pix2xyz(depth_img, i, j):
        return [
            depth_img[i, j] * j * 1000,
            depth_img[i, j] * i * 1000,
            depth_img[i, j] * 1000
        ]