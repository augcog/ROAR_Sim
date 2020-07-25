from ROAR_simulation.roar_autonomous_system.planning_module.abstract_planner\
    import (
    AbstractPlanner,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import (
    Vehicle,
)
from typing import Any


class BehaviorPlanner(AbstractPlanner):
    def run_step(self, vehicle: Vehicle) -> Any:
        pass
