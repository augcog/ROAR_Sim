from ROAR.roar_autonomous_system.agent_module.agent import Agent
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.roar_autonomous_system.configurations.agent_settings import AgentConfig
from ROAR.roar_autonomous_system.planning_module.mission_planner.json_waypoint_planner import JSONWaypointPlanner
from ROAR.roar_autonomous_system.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.roar_autonomous_system.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.roar_autonomous_system.control_module.pure_pursuit_control import PurePursuitController


class JsonWaypointAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle, agent_settings)
        self.controller = PurePursuitController(agent=self)
        self.mission_planner = JSONWaypointPlanner(self)
        self.behavior_planner = BehaviorPlanner(self)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(agent=self,
                                                                 mission_planner=self.mission_planner,
                                                                 behavior_planner=self.behavior_planner,
                                                                 controller=self.controller)
        self.logger.debug("JSON Waypoint Agent Initialized")

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super(JsonWaypointAgent, self).run_step(sensors_data=sensors_data, vehicle=vehicle)
        next_control = self.local_planner.run_step()

        return next_control
