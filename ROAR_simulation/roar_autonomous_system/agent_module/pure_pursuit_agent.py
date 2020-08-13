from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module \
    .data_structures_models import \
    SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import \
    Vehicle, VehicleControl
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module \
    .pure_pursuit_control import \
    PurePursuitController
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner \
    .simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings \
    import \
    AgentConfig
from ROAR_simulation.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import img_to_world
import numpy as np
import cv2
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import png_to_depth
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform, Location, Rotation

class PurePursuitAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, target_speed=50):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings)
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pure_pursuit_controller = \
            PurePursuitController(vehicle=vehicle,
                                  target_speed=target_speed,
                                  look_ahead_gain=0.1,
                                  look_ahead_distance=1)
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=vehicle)

        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=vehicle,
            controller=self.pure_pursuit_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.visualizer = Visualizer(agent=self)
        self.occupancy_grid_map = OccupancyGridMap(
            absolute_maximum_map_size=self.agent_settings.absolute_maximum_map_size
        )

    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(PurePursuitAgent, self).run_step(sensors_data=sensors_data,
                                               vehicle=vehicle)
        try:
            print("***************")
            waypoint0 = self.local_planner.way_points_queue[0]
            waypoint1 = self.local_planner.way_points_queue[1]
            waypoint2 = self.local_planner.way_points_queue[2]
            custom_point = Transform(
                location=Location(x=2, y=39.23454284667969, z=-0.005509738810360432)
            )
            img_pos_0 = self.visualizer.calculate_img_pos(waypoint_transform=waypoint0, camera=self.front_depth_camera)
            img_pos_1 = self.visualizer.calculate_img_pos(waypoint_transform=waypoint1, camera=self.front_depth_camera)
            img_pos_2 = self.visualizer.calculate_img_pos(waypoint_transform=waypoint2, camera=self.front_depth_camera)
            img_pos_custom = self.visualizer.calculate_img_pos(waypoint_transform=custom_point, camera=self.front_depth_camera)

            depth = self.front_depth_camera.data
            depth_0 = depth[img_pos_0[1]][img_pos_0[0]] * 1000
            depth_1 = depth[img_pos_1[1]][img_pos_1[0]] * 1000
            depth_2 = depth[img_pos_2[1]][img_pos_2[0]] * 1000
            depth_custom = depth[img_pos_custom[1]][img_pos_custom[0]] * 1000

            # print(img_pos_0, depth_0,
            #       (np.linalg.norm(waypoint0.location.to_array() - self.vehicle.transform.location.to_array()) - 1.5))
            # print(img_pos_1, depth_1,
            #       (np.linalg.norm(waypoint1.location.to_array() - self.vehicle.transform.location.to_array()) - 1.5))
            # print(img_pos_2, depth_2,
            #       (np.linalg.norm(waypoint2.location.to_array() - self.vehicle.transform.location.to_array()) - 1.5))

            rgb = self.front_rgb_camera.data.copy()
            rgb[img_pos_0[1]:img_pos_0[1] + 5, img_pos_0[0]:img_pos_0[0] + 5] = [0, 255, 0]
            rgb[img_pos_1[1]:img_pos_1[1] + 5, img_pos_1[0]:img_pos_1[0] + 5] = [0, 255, 0]
            rgb[img_pos_2[1]:img_pos_2[1] + 5, img_pos_2[0]:img_pos_2[0] + 5] = [0, 255, 0]
            rgb[img_pos_custom[1]:img_pos_custom[1] + 5, img_pos_custom[0]:img_pos_custom[0] + 5] = [0, 255, 0]
            cv2.imshow("image", rgb)
            cv2.waitKey(1)
            # #
            # depth_array = self.front_depth_camera.data.copy()
            # depth_array[depth_array > 0.089] = 0
            # depth_array = depth_array / np.amax(depth_array)
            # cv2.imshow("depthimg", depth_array)
            # cv2.waitKey(1)

            raw_p2d = np.array([
                [img_pos_0[0] * depth_0, img_pos_0[1] * depth_0, depth_0],
                [img_pos_1[0] * depth_1, img_pos_1[1] * depth_1, depth_1],
                [img_pos_2[0] * depth_2, img_pos_2[1] * depth_2, depth_2],
                [img_pos_custom[0] * depth_custom, img_pos_custom[1] * depth_custom, depth_custom]
            ])
            # print(raw_p2d)
            cords_y_minus_z_x = np.linalg.inv(self.front_depth_camera.intrinsics_matrix) @ raw_p2d.T
            cords_xyz_1 = np.vstack([
                cords_y_minus_z_x[2, :],
                cords_y_minus_z_x[0, :],
                -cords_y_minus_z_x[1, :],
                np.ones((1, np.shape(cords_y_minus_z_x)[1]))
            ])
            # print(cords_xyz_1)
            points = self.vehicle.transform.get_matrix() @ self.front_depth_camera.transform.get_matrix() @ cords_xyz_1
            calculated = points.T[:, :3]
            truth = np.array([waypoint0.location.to_array(), waypoint1.location.to_array(), waypoint2.location.to_array(), custom_point.location.to_array()])
            print(np.linalg.norm(calculated - truth, axis=1))
            print(img_pos_0, img_pos_1, img_pos_2, img_pos_custom)

        except Exception as e:
            print(f"ERROR: {e}")

        # print()
        return self.local_planner.run_step(vehicle=vehicle)
