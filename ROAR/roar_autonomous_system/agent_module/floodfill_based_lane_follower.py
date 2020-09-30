from ROAR.roar_autonomous_system.agent_module.agent import Agent, AgentConfig
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.roar_autonomous_system.control_module.pid_controller import VehiclePIDController
from ROAR.roar_autonomous_system.perception_module.flood_fill_lane_detector import FloodfillLaneDetector
from ROAR.roar_autonomous_system.control_module.pid_controller import PIDParam


class FloodfillBasedLaneFollower(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle, agent_settings)
        self.controller = VehiclePIDController(agent=self, args_lateral=PIDParam.default_lateral_param(),
                                               args_longitudinal=PIDParam.default_longitudinal_param())
        self.floodfill_lane_detector = FloodfillLaneDetector(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super().run_step(sensors_data=sensors_data, vehicle=vehicle)
        self.floodfill_lane_detector.run_step()
        return VehicleControl()
