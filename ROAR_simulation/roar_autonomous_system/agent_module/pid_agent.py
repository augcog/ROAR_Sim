from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    VehiclePIDController
from ROAR_simulation.roar_autonomous_system.control_module.mpc_controller \
    import \
    VehicleMPCController
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner\
    .simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner\
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner\
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    PIDParam
from ROAR_simulation.roar_autonomous_system.utilities_module\
    .data_structures_models import \
    SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import \
    VehicleControl, Vehicle
import logging
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer

from ROAR_simulation.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import img_to_world

class PIDAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.logger = logging.getLogger("PID Agent")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pid_controller = VehiclePIDController(
            vehicle=self.vehicle,
            args_lateral=PIDParam.default_lateral_param(),
            args_longitudinal=PIDParam.default_longitudinal_param(),
            target_speed=target_speed)
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
        self.visualizer = Visualizer(agent=self)

        self.occupancy_grid_map = OccupancyGridMap(min_x=-500,
                                                   min_y=-500,
                                                   max_x=500,
                                                   max_y=500)

        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")
        self.curr_max_err = 0
        self.counter = 0
        self.total_err = 0

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(PIDAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)
        return control
