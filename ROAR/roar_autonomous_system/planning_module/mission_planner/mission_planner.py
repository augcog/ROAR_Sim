from typing import List
import logging
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import (
    Transform,
)
from collections import deque
from ROAR.roar_autonomous_system.planning_module.abstract_planner import (
    AbstractPlanner,
)
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
)


class MissionPlanner(AbstractPlanner):
    def __init__(self, vehicle: Vehicle):
        super().__init__(vehicle=vehicle)
        self.logger = logging.getLogger(__name__)
        self.mission_plan: deque = deque()

    def run_step(self, vehicle: Vehicle) -> List[Transform]:
        """
        Abstract run step function for Mission Planner

        Args:
            vehicle: new vehicle state

        Returns:
            Plan for next steps

        """
        return []
