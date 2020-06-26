from abc import ABC, abstractmethod
import logging
from typing import List
from roar_autonomous_system.util.models import Transform
from roar_autonomous_system.planning.planner import Planer


class BehaviorPlanner(Planer):
    def __init__(self, mission_path: List[Transform] = None):
        super().__init__()
        self.mission_path: List[Transform] = [] if mission_path is None else mission_path
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def generate_constraints(self):
        pass
