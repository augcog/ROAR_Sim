from roar_autonomous_system.planning.behavior_planners.behavior_planner import BehaviorPlanner
from typing import List, Optional
from roar_autonomous_system.util.models import Transform


class NoActionBehaviorPlanner(BehaviorPlanner):
    def __init__(self, mission_path: Optional[List[Transform]] = None):
        super().__init__(mission_path=mission_path)
        self.logger.debug("No Action Behavior Planner Initiated")

    def generate_constraints(self):
        return None

    def sync(self):
        pass
