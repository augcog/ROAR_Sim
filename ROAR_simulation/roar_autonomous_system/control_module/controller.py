from abc import ABC, abstractmethod
import logging

from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
    VehicleControl,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform


class Controller(ABC):
    def __init__(self, vehicle: Vehicle):
        """

        Args:
            vehicle: Vehicle instance

        """
        self.vehicle = vehicle
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def run_step(self, vehicle: Vehicle, next_waypoint: Transform, **kwargs) \
            -> VehicleControl:
        """
        Abstract function for run step

        Args:
            vehicle: new vehicle state
            next_waypoint: next waypoint
            **kwargs:

        Returns:
            VehicleControl
        """
        self.sync_data(vehicle=vehicle)
        return VehicleControl()

    def sync_data(self, vehicle: Vehicle) -> None:
        """
        default sync data function
        Args:
            vehicle: new vehicle state

        Returns:
            None

        """
        self.vehicle = vehicle
