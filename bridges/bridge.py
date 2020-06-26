"""
This file defines a basic Bridge for extensibility of the ROAR Autonomous software

"""
from abc import abstractmethod, ABC
import logging
from roar_autonomous_system.util.models import Control, SensorData
from typing import Any


class Bridge(ABC):
    def __init__(self):
        self.logger = logging.Logger(__name__)

    @abstractmethod
    def convert_control_data_from_source_to_agent(self, source) -> Control:
        return Control.parse_obj(source)

    @abstractmethod
    def convert_control_data_from_agent_to_source(self, control: Control) -> Any:
        return control

    @abstractmethod
    def convert_sensor_data_from_source_to_agent(self, source) -> SensorData:
        return SensorData.parse_obj(source)


