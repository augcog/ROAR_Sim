from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import (
    SensorsData,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
    VehicleControl,
)
from pathlib import Path
from ROAR_simulation.roar_autonomous_system.control_module.pure_pursuit_control import (
    PurePursuitController,
)
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner.waypoint_following_mission_planner import (
    WaypointFollowingMissionPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner.behavior_planner import (
    BehaviorPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import (
    SimpleWaypointFollowingLocalPlanner,
)
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer import (
    Visualizer,
)


class PurePursuitAgent(Agent):
    def __init__(self, vehicle: Vehicle, route_file_path: Path):
        super().__init__(vehicle)
        self.route_file_path = route_file_path
        self.pure_pursuit_controller = PurePursuitController(
            vehicle=vehicle,
            target_speed=150,
            look_ahead_gain=0.1,
            look_ahead_distance=1,
        )
        self.mission_planner = WaypointFollowingMissionPlanner(
            file_path=self.route_file_path, vehicle=vehicle
        )

        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(vehicle=vehicle)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            vehicle=vehicle,
            controller=self.pure_pursuit_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1,
        )
        self.visualizer = Visualizer(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(PurePursuitAgent, self).run_step(
            sensors_data=sensors_data, vehicle=vehicle
        )
        return self.local_planner.run_step(vehicle=vehicle)
