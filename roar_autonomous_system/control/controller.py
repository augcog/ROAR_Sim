import logging
from abc import ABC, abstractmethod
from roar_autonomous_system.util.models import Control, Vehicle, Transform


class Controller(ABC):
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.logger = logging.Logger(__name__)

    @abstractmethod
    def run_step(self, next_waypoint: Transform) -> Control:
        return Control()
