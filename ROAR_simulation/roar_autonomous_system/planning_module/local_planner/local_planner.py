from abc import abstractmethod
from ROAR_simulation.roar_autonomous_system.planning_module.abstract_planner import (
    AbstractPlanner,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
)
from ROAR_simulation.roar_autonomous_system.control_module.controller import Controller
from ROAR_simulation.roar_autonomous_system.planning_module.behavior_planner.behavior_planner import (
    BehaviorPlanner,
)
from ROAR_simulation.roar_autonomous_system.planning_module.mission_planner.mission_planner import (
    MissionPlanner,
)
from typing import Optional
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    VehicleControl,
)
from collections import deque


class LocalPlanner(AbstractPlanner):
    def __init__(
        self,
        vehicle: Vehicle,
        controller: Optional[Controller] = None,
        behavior_planner: Optional[BehaviorPlanner] = None,
        mission_planner: Optional[MissionPlanner] = None,
    ):
        super().__init__(vehicle)
        self.controller = (
            Controller(vehicle=vehicle) if controller is None else controller
        )
        self.behavior_planner = (
            BehaviorPlanner(vehicle=vehicle)
            if behavior_planner is None
            else behavior_planner
        )
        self.mission_planner = (
            MissionPlanner(vehicle=vehicle)
            if mission_planner is None
            else mission_planner
        )
        self.way_points_queue = deque()

    @abstractmethod
    def run_step(self, vehicle: Vehicle) -> VehicleControl:
        return VehicleControl()
