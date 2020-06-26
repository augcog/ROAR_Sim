from abc import ABC, abstractmethod
import logging
from typing import List
from roar_autonomous_system.util.models import Transform


class BehaviorPlanner(ABC):
    def __init__(self, mission_path: List[Transform] = None):
        self.mission_path: List[Transform] = [] if mission_path is None else mission_path
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def generate_constraints(self):
        pass
