from abc import ABC, abstractmethod
from typing import List
import logging
from roar_autonomous_system.util.models import Transform
from collections import deque


class MissionPlanner(ABC):
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.mission_plan: deque = deque()

    @abstractmethod
    def update_mission_plan(self) -> List[Transform]:
        pass
