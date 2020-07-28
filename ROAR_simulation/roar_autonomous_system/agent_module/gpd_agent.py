from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings import AgentConfig
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.perception_module.gpd_detector import GroundPlaneDetector


class GPDAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings)
        self.gpd_detector = GroundPlaneDetector(
            agent=self
        )

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(GPDAgent, self).run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.gpd_detector.run_step()
        return VehicleControl()
