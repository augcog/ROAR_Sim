from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    VehiclePIDController
from ROAR_simulation.roar_autonomous_system.control_module.mpc_controller \
    import \
    VehicleMPCController
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner \
    .simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner \
    .behavior_planner import \
    BehaviorPlanner
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner \
    .waypoint_following_mission_planner import \
    WaypointFollowingMissionPlanner
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    PIDParam
from ROAR_simulation.roar_autonomous_system.utilities_module \
    .data_structures_models import \
    SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import \
    VehicleControl, Vehicle
import logging
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer


class MPCAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.logger = logging.getLogger("PathFollowingAgent")
        self.mpc_controller = VehicleMPCController(
            vehicle=self.vehicle,
            route_file_path=Path(
                self.agent_settings.waypoint_file_path),
            target_speed=target_speed)
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=Path(self.agent_settings.waypoint_file_path),
            vehicle=self.vehicle,)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=self.vehicle,)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=self.vehicle,
            controller=self.mpc_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.visualizer = Visualizer(agent=self)
        self.logger.debug(f"Waypoint Following Agent Initiated. "
                          f"Reading from "
                          f"{self.agent_settings.waypoint_file_path}")
        self.curr_max_err = 0
        self.counter = 0
        self.total_err = 0

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(MPCAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        if self.local_planner.is_done():
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_step(vehicle=vehicle)
        return control
