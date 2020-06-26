"""
This file defines a basic Bridge for extensibility of the ROAR Autonomous software

"""
from abc import abstractmethod, ABC
import logging
from roar_autonomous_system.util.models import Location, Rotation, Transform, Control, RGBData, DepthData, \
    Vector3D, IMUData, SensorData, Vehicle
from typing import Any


class Bridge(ABC):
    def __init__(self):
        self.logger = logging.Logger(__name__)

    """ 
        INPUT 
    """

    @abstractmethod
    def convert_location_from_source_to_agent(self, source) -> Location:
        pass

    @abstractmethod
    def convert_rotation_from_source_to_agent(self, source) -> Rotation:
        pass

    @abstractmethod
    def convert_transform_from_source_to_agent(self, source) -> Transform:
        pass

    @abstractmethod
    def convert_control_from_source_to_agent(self, source) -> Control:
        pass

    @abstractmethod
    def convert_rgb_from_source_to_agent(self, source) -> RGBData:
        pass

    @abstractmethod
    def convert_depth_from_source_to_agent(self, source) -> DepthData:
        pass

    @abstractmethod
    def convert_vector3d_from_source_to_agent(self, source) -> Vector3D:
        pass

    @abstractmethod
    def convert_imu_from_source_to_agent(self, source) -> IMUData:
        pass

    @abstractmethod
    def convert_sensor_data_from_source_to_agent(self, source) -> SensorData:
        pass

    @abstractmethod
    def convert_vehicle_from_source_to_agent(self, source) -> Vehicle:
        pass

    """ 
        OUTPUT 
    """

    @abstractmethod
    def convert_control_from_agent_to_source(self, control: Control) -> Any:
        pass

    @abstractmethod
    def convert_vector3d_from_agent_to_source(self, vector3d: Vector3D) -> Any:
        pass

    @abstractmethod
    def convert_vehicle_from_agent_to_source(self, vehicle: Vehicle) -> Any:
        pass
