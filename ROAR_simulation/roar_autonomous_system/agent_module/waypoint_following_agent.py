from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller import VehiclePIDController
from ROAR_simulation.roar_autonomous_system.control_module.mpc_controller import VehicleMPCController
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner.waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller import PIDParam
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import VehicleControl, Vehicle
import logging
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer import Visualizer


class WaypointFollowingAgent(Agent):
    def __init__(self, vehicle, route_file_path: Path, target_speed=40, **kwargs):
        super().__init__(vehicle, **kwargs)
        self.logger = logging.getLogger("PathFollowingAgent")
        self.route_file_path = route_file_path
        self.pid_controller = VehiclePIDController(vehicle=vehicle,
                                                   args_lateral=PIDParam.default_lateral_param(),
                                                   args_longitudinal=PIDParam.default_longitudinal_param(),
                                                   target_speed=target_speed)
        self.mpc_controller = VehicleMPCController(vehicle=vehicle,
                                                   route_file_path=route_file_path,
                                                   target_speed=target_speed)
        self.mission_planner = WaypointFollowingMissionPlanner(file_path=self.route_file_path, vehicle=vehicle)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(vehicle=vehicle,
                                                                #  controller=self.pid_controller,
                                                                 controller=self.mpc_controller,
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
        return control
