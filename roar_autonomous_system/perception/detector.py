from abc import ABC, abstractmethod
import logging
from roar_autonomous_system.util.models import Vehicle

class Detector(ABC):
    def __init__(self, vehicle:Vehicle):
        self.logger = logging
        self.logger = logging.getLogger(__name__)
        self.vehicle: Vehicle = vehicle

    @abstractmethod
    def run_step(self):
        """
        This method needs to be called on every step

        Returns:

        """
        self.sync()
        return None

    @abstractmethod
    def sync(self):
        return None
