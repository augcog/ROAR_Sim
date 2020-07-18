from ROAR_simulation.carla_client.settings import CarlaSettings
import logging
import pygame
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from typing import Tuple
from ROAR_simulation.bridges.carla_bridge import CarlaBridge
import carla
from ROAR_simulation.carla_client.util.hud import HUD
from ROAR_simulation.carla_client.util.world import World
from ROAR_simulation.carla_client.util.keyboard_control import KeyboardControl


class CarlaRunner:

    def __init__(self, carla_settings: CarlaSettings):

        self.carla_settings = carla_settings
        self.carla_bridge = CarlaBridge()
        self.world = None
        self.client = None
        self.controller = None
        self.display = None

        self.agent = None
        self.logger = logging.getLogger(__name__)

    def set_carla_world(self) -> Vehicle:
        """Initiating the vehicle with loading messages"""

        try:

            pygame.init()
            pygame.font.init()
            self.logger.debug(f"Connecting to {self.carla_settings.host}: {self.carla_settings.port}")
            self.client = carla.Client(self.carla_settings.host, self.carla_settings.port)
            self.client.set_timeout(2.0)

            self.display = pygame.display.set_mode((self.carla_settings.width, self.carla_settings.height),
                                                   pygame.HWSURFACE | pygame.DOUBLEBUF)

            self.logger.debug(f"Setting HUD")
            hud = HUD(self.carla_settings.width, self.carla_settings.height)

            self.logger.debug("Setting up world")
            self.world = World(carla_world=self.client.get_world(), hud=hud, settings=self.carla_settings)

            self.logger.debug(f"Connecting to Keyboard controls")
            self.controller = KeyboardControl(world=self.world, carla_setting=self.carla_settings)
            self.logger.debug("All settings done")

            return self.carla_bridge.convert_vehicle_from_source_to_agent(self.world.player)

        except Exception as e:
            self.logger.error(f"Unable to initiate the world due to error: {e}")
            raise e

    def start_game_loop(self, agent: Agent, use_manual_control=False):
        """Start running the vehicle and stop when finished running the track"""

        self.agent = agent
        try:
            self.logger.debug("Initiating game")
            clock = pygame.time.Clock()
            while True:
                # make sure the program does not run above 40 frames per second
                # this allow proper synchrony between server and client
                clock.tick_busy_loop(60)
                should_continue, carla_control = self.controller.parse_events(client=self.client,
                                                                              world=self.world,
                                                                              clock=clock)

                if not should_continue:
                    break
                self.world.tick(clock)
                self.world.render(display=self.display)
                pygame.display.flip()
                sensor_data, new_vehicle = self.convert_data()

                if self.carla_settings.enable_autopilot:
                    if self.agent is None:
                        raise Exception("In autopilot mode, but no agent is defined.")
                    agent_control = self.agent.run_step(vehicle=new_vehicle, sensors_data=sensor_data)
                    if not use_manual_control:
                        carla_control = self.carla_bridge.convert_control_from_agent_to_source(agent_control)
                self.world.player.apply_control(carla_control)
        except Exception as e:
            self.logger.error(f"Error happened, exiting safely. Error: {e}")

        finally:
            self.logger.debug("Ending Game")
            if self.world is not None:
                self.world.destroy()
                self.logger.debug("All actors are destroyed")
            try:
                pygame.quit()
            except Exception as e:
                self.logger.debug(f"Cannot quit pygame normally, force quitting. Error: {e}")
            self.logger.debug("Game ended")
            exit(0)

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        """
        Convert data from source to agent

        Returns:
            sensors_data: sensor data for agent
            new_vehicle: the current player's vehicle state

        """
        sensor_data: SensorsData = self.carla_bridge.convert_sensor_data_from_source_to_agent(
            {
                "front_rgb": None if self.world.front_rgb_sensor_data is None else self.world.front_rgb_sensor_data,
                "rear_rgb": None if self.world.rear_rgb_sensor_data is None else self.world.rear_rgb_sensor_data,
                "front_depth": None if self.world.front_depth_sensor_data is None else
                self.world.front_depth_sensor_data,
                "imu": self.world.imu_sensor
            }
        )
        new_vehicle = self.carla_bridge.convert_vehicle_from_source_to_agent(self.world.player)
        return sensor_data, new_vehicle

