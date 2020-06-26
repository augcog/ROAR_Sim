import logging
from abc import ABC, abstractmethod
from roar_autonomous_system.util.models import Control, Vehicle
from bridges.bridge import Bridge

class Agent(ABC):
    def __init__(self, vehicle: Vehicle, bridge:Bridge):
        self.logger = logging.Logger(__name__)
        self.bridge = bridge
        self.vehicle = vehicle
        self.mission_planner = None
        self.behavior_planner = None
        self.local_planner = None

    @abstractmethod
    def run_step(self) -> Control:
        self.vehicle.control = Control()
        return self.vehicle.control
