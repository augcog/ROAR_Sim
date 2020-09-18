from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_Jetson.jetson_vehicle import Vehicle as JetsonVehicle
from typing import Optional, Tuple
from ROAR_Jetson.controller import NaiveController
from ROAR_Jetson.jetson_cmd_sender import JetsonCommandSender
from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.bridges.jetson_bridge import JetsonBridge
from ROAR_Jetson.camera import RS_D435i
import logging
import time


class JetsonRunner:
    def __init__(self, agent: Agent):
        self.jetson_vehicle: JetsonVehicle = JetsonVehicle()
        self.agent = agent
        self.jetson_bridge = JetsonBridge()
        self.logger = logging.getLogger("Jetson Runner")
        self.setup_jetson_vehicle()
        self.logger.info("Jetson Vehicle Connected and Intialized")

    def setup_jetson_vehicle(self):
        self.jetson_vehicle.add(JetsonCommandSender(), inputs=['throttle', 'steering'], threaded=True)
        self.jetson_vehicle.add(RS_D435i(), threaded=True)

    def start_game_loop(self, use_manual_control=False, rate_hz=40):
        self.logger.info("Starting Game Loop")
        try:
            while True:
                start_time = time.time()
                # pass throttle and steering into the bridge
                sensors_data, vehicle = self.convert_data()
                # run a step of agent
                vehicle_control: VehicleControl = self.agent.run_step(sensors_data=sensors_data, vehicle=vehicle)
                # pass the output into sender to send it
                self.jetson_vehicle.update_parts(new_throttle=vehicle_control.throttle,
                                                 new_steering=vehicle_control.steering)

                # end_time = time.time()
                # diff_time = end_time - start_time
                # if diff_time < 1 / rate_hz:
                #     time.sleep(diff_time)

        except Exception as e:
            self.logger.error(f"Something bad happened: {e}")
        finally:
            self.jetson_vehicle.stop()

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        sensors_data: SensorsData = self.jetson_bridge.convert_sensor_data_from_source_to_agent(
            source={
                "front_rgb": self.jetson_vehicle.front_rgb_img,
                "rear_rgb": None,
                "front_depth": self.jetson_vehicle.front_depth_img,
                "imu": None
            }
        )
        new_vehicle = self.jetson_bridge.convert_vehicle_from_source_to_agent(self.jetson_vehicle)
        return sensors_data, new_vehicle
