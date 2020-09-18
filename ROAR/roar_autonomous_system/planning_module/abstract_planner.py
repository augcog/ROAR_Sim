from abc import ABC, abstractmethod
import logging
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
)
from typing import Any


class AbstractPlanner(ABC):
    def __init__(self, vehicle: Vehicle):
        self.logger = logging
        self.logger = logging.getLogger(__name__)
        self.vehicle: Vehicle = vehicle

    @abstractmethod
    def run_step(self, vehicle: Vehicle) -> Any:
        """
        On every step, produce an actionable plan
        Returns:
        """
        self.sync_data(vehicle=vehicle)
        return None

    def sync_data(self, vehicle: Vehicle):
        """
        Sync the incoming vehicle state

        Args:
            vehicle: new frame's vehicle state

        Returns:
            None

        """
        self.vehicle = vehicle
