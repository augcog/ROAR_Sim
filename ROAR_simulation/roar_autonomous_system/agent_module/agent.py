from abc import ABC, abstractmethod
import logging
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR_simulation.roar_autonomous_system.utilities_module.camera_models import Camera
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData, IMUData, Transform
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import VehicleControl
from typing import Optional, List
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings import AgentSettings


class Agent(ABC):
    """
    Abstract Agent class that define the minimum of a ROAR agent.

    Inherited agent can perform different duties.

    """

    def __init__(self,
                 vehicle: Vehicle,
                 agent_settings: AgentSettings,
                 imu: Optional[IMUData] = None):
        """Initiating the Agent with given vehicle, front and back RGB cameras, front depth camera and IMU sensor"""

        self.vehicle = vehicle
        self.agent_settings = agent_settings
        self.front_rgb_camera = agent_settings.front_rgb_cam
        self.front_depth_camera = agent_settings.front_depth_cam
        self.rear_rgb_camera = agent_settings.rear_rgb_cam

        self.init_cam()

        self.imu = imu
        self.logger = logging.getLogger(__name__)
        self.transform_history: List[Transform] = []

    def init_cam(self):
        """Calculate intrinsic matrices for each existing camera (front/back RGB and front depth). """

        if self.front_rgb_camera is not None:
            self.front_rgb_camera.intrinsics_matrix = self.front_rgb_camera.calculate_intrinsic_matrix()
        if self.front_depth_camera is not None:
            self.front_depth_camera.intrinsics_matrix = self.front_depth_camera.calculate_intrinsic_matrix()
        if self.rear_rgb_camera is not None:
            self.rear_rgb_camera.intrinsics_matrix = self.rear_rgb_camera.calculate_intrinsic_matrix()


    @abstractmethod
    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        """
        Receive Sensor Data and vehicle state information on every step and return a control

        Args:
            sensors_data: sensor data on this frame
            vehicle: vehicle state on this frame

        Returns:
            Vehicle Control

        """
        self.sync_data(sensors_data=sensors_data, vehicle=vehicle)
        return VehicleControl()

    def sync_data(self, sensors_data: SensorsData, vehicle: Vehicle):
        """Syncing the data from cameras with the ROAR Agent's camera data. """

        self.vehicle = vehicle

        if self.front_rgb_camera is not None:
            self.front_rgb_camera.data = sensors_data.front_rgb.data if sensors_data.front_rgb is not None else None

        if self.front_depth_camera is not None:
            self.front_depth_camera.data = sensors_data.front_depth.data if sensors_data.front_depth is not None else None

        if self.rear_rgb_camera is not None:
            self.rear_rgb_camera.data = sensors_data.rear_rgb.data if sensors_data.rear_rgb is not None else None

        if self.imu is not None:
            self.imu = sensors_data.imu_data
