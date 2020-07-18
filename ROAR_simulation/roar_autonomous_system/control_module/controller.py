from abc import ABC, abstractmethod
import logging

from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl


class Controller(ABC):
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def run_step(self, vehicle: Vehicle, next_waypoint, **kwargs) -> VehicleControl:
        self.sync_data(vehicle=vehicle)
        return VehicleControl()

    def sync_data(self, vehicle):
        self.vehicle = vehicle
