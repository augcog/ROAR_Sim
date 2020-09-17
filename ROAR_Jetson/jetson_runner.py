from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_Jetson.jetson_vehicle import Vehicle as JetsonVehicle
from typing import Optional, Tuple
from ROAR_Jetson.controller import NaiveController
from ROAR_Jetson.jetson_cmd_sender import JetsonCommandSender
from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.bridges.jetson_bridge import JetsonBridge
import logging

class JetsonRunner:
    def __init__(self, agent: Agent):
        self.jetson_vehicle: JetsonVehicle = JetsonVehicle()
        self.agent = agent
        self.jetson_bridge = JetsonBridge()
        self.logger = logging.getLogger("Jetson Runner")

    def setup_jetson_vehicle(self):
        self.jetson_vehicle.add(JetsonCommandSender(), inputs=['throttle', 'steering'], threaded=True)

    def start_game_loop(self, use_manual_control=False):
        try:
            while True:
                # pass throttle and steering into the bridge
                sensors_data, vehicle = self.convert_data()
                # run a step of agent
                vehicle_control: VehicleControl = self.agent.run_step(sensors_data=sensors_data, vehicle=vehicle)
                # pass the output into sender to send it
                self.jetson_vehicle.update_parts(new_throttle=vehicle_control.throttle, new_steering=vehicle_control.steering)

        except Exception as e:
            self.logger.error(f"Something bad happened: {e}")
        finally:
            self.jetson_vehicle.stop()

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        sensors_data: SensorsData = self.jetson_bridge.convert_sensor_data_from_source_to_agent(
            source={
                "front_rgb": None,
                "rear_rgb": None,
                "front_depth": None,
                "imu": None
            }
        )
        new_vehicle = self.jetson_bridge.convert_vehicle_from_source_to_agent(self.jetson_vehicle)
        return sensors_data, new_vehicle
