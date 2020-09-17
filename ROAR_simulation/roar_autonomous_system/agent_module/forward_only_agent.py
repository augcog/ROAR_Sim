from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings import AgentConfig
import numpy as np
import random


class ForwardOnlyAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle, agent_settings)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        control = VehicleControl(throttle=0.5, steering=0)
        self.logger.debug(f"Outputting {control}")
        return control
