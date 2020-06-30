import logging
from abc import ABC, abstractmethod
from roar_autonomous_system.util.models import Control, Vehicle, RGBData, DepthData, SensorData, Transform, OccupancyMap
from bridges.bridge import Bridge
from typing import Union, List
import numpy as np


class Agent(ABC):
    def __init__(self, vehicle: Vehicle, bridge: Bridge, visualize_occupancy_map=True):
        self.bridge = bridge
        self.vehicle = vehicle
        self.mission_planner = None
        self.behavior_planner = None
        self.local_planner = None
        self.sensor_data: Union[None, SensorData] = None
        self.global_occupancy_grid_map: Union[None, OccupancyMap] = None
        self.visualize_occupancy_map = visualize_occupancy_map

        # for logging and debugging purposes
        self.logger = logging.Logger(__name__)
        self.transform_history: List[Transform] = []

    @abstractmethod
    def run_step(self, vehicle: Vehicle, sensor_data: SensorData) -> Control:
        self.sync(vehicle=vehicle, sensor_data=sensor_data)
        self.transform_history.append(self.vehicle.transform)
        self.vehicle.control = Control()
        return self.vehicle.control

    @abstractmethod
    def sync(self, vehicle: Vehicle, sensor_data: SensorData):
        self.vehicle = vehicle
        self.sensor_data = sensor_data

