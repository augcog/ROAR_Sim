import carla
from carla import ColorConverter as cc
from ROAR_Sim.carla_client.util.sensors import CollisionSensor
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig
import logging
import pygame
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle
from typing import Tuple
from Bridges.carla_bridge import CarlaBridge
from ROAR_Sim.carla_client.util.hud import HUD
from ROAR_Sim.carla_client.util.world import World
from ROAR_Sim.carla_client.util.keyboard_control import KeyboardControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path
from typing import List, Dict, Any
from ROAR.utilities_module.vehicle_models import VehicleControl
import json
from typing import Optional
import numpy as np

class CarlaRunner:

    def __init__(self,
                 carla_settings: CarlaConfig,
                 agent_settings: AgentConfig,
                 npc_agent_class, competition_mode=False, max_collision=1000):
        """

        Args:
            carla_settings:
            agent_settings:
            npc_agent_class:
            competition_mode: if True, will exist when max_collision is reached
            max_collision: number of maximum collision allowed
        """
        self.carla_settings = carla_settings
        self.agent_settings = agent_settings
        self.carla_bridge = CarlaBridge()
        self.npc_agent_class = npc_agent_class
        self.world = None
        self.client = None
        self.controller = None
        self.display = None
        self.agent = None

        self.npc_agents: Dict[npc_agent_class, Any] = {}
        self.agent_collision_counter = 0

        self.competition_mode = competition_mode
        self.max_collision = max_collision
        self.start_simulation_time: Optional[float] = None
        self.start_vehicle_position: Optional[np.array] = None
        self.end_simulation_time: Optional[float] = None
        self.end_vehicle_position: Optional[np.array] = None

        self.logger = logging.getLogger(__name__)
        self.timestep_counter = 0

    def set_carla_world(self) -> Vehicle:
        """
        Initiating the vehicle with loading messages
        Returns:
            Vehicle Information
        """

        try:
            pygame.init()
            pygame.font.init()
            self.logger.debug(f"Connecting to {self.carla_settings.host}: "
                              f"{self.carla_settings.port}")
            self.client = carla.Client(self.carla_settings.host,
                                       self.carla_settings.port)
            self.client.set_timeout(self.carla_settings.timeout)

            self.display = pygame.display.set_mode(
                (self.carla_settings.width, self.carla_settings.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

            self.logger.debug(f"Setting HUD")
            hud = HUD(self.carla_settings.width, self.carla_settings.height)

            self.logger.debug("Setting up world")
            self.world = World(carla_world=self.client.get_world(), hud=hud,
                               carla_settings=self.carla_settings,
                               agent_settings=self.agent_settings)

            if self.carla_settings.should_spawn_npcs:
                self.spawn_npcs()

            self.logger.debug(f"Connecting to Keyboard controls")
            self.controller = KeyboardControl(world=self.world,
                                              carla_setting=self.carla_settings
                                              )
            self.logger.debug("All settings done")

            return self.carla_bridge. \
                convert_vehicle_from_source_to_agent(self.world.player)

        except Exception as e:
            self.logger.error(
                f"Unable to initiate the world due to error: {e}")
            raise e

    def start_game_loop(self, agent, use_manual_control=False, max_timestep=1e20):
        """Start running the vehicle and stop when finished running
        the track"""

        self.agent = agent
        try:
            self.logger.debug("Initiating game")
            self.agent.start_module_threads()
            clock = pygame.time.Clock()
            self.start_simulation_time = self.world.hud.simulation_time
            self.start_vehicle_position = self.agent.vehicle.transform.location.to_array()
            while True and self.timestep_counter < max_timestep:

                # make sure the program does not run above 60 frames per second
                # this allow proper synchrony between server and client
                clock.tick_busy_loop(60)
                should_continue, carla_control = self.controller.parse_events(client=self.client,
                                                                              world=self.world,
                                                                              clock=clock)

                collision_sensor: CollisionSensor = self.world.collision_sensor

                if self.competition_mode and len(collision_sensor.history) > self.max_collision:
                    self.agent_collision_counter = len(collision_sensor.history)
                    should_continue = False

                if not should_continue:
                    break

                self.world.tick(clock)
                self.world.render(display=self.display)
                pygame.display.flip()
                sensor_data, new_vehicle = self.convert_data()
                if self.carla_settings.save_semantic_segmentation and self.world.semantic_segmentation_sensor_data:
                    self.world.semantic_segmentation_sensor_data.save_to_disk((Path(
                        "./data/output") / "ss" / f"frame_{self.agent.time_counter}.png").as_posix(),
                                                                              cc.CityScapesPalette)

                if self.carla_settings.should_spawn_npcs:
                    self.execute_npcs_step()

                if self.agent_settings.enable_autopilot:
                    if self.agent is None:
                        raise Exception(
                            "In autopilot mode, but no agent is defined.")
                    agent_control = self.agent.run_step(vehicle=new_vehicle,
                                                        sensors_data=sensor_data)
                    if not use_manual_control:
                        carla_control = self.carla_bridge. \
                            convert_control_from_agent_to_source(agent_control)
                self.world.player.apply_control(carla_control)

                self.timestep_counter += 1
        except Exception as e:
            self.logger.error(f"Error happened, exiting safely. Error: {e}")

        finally:
            self.on_finish()

    def on_finish(self):
        self.logger.debug("Ending Game")
        self.end_simulation_time = self.world.hud.simulation_time
        self.end_vehicle_position = self.agent.vehicle.transform.location.to_array()
        if self.world is not None:
            self.world.destroy()
            self.logger.debug("All actors are destroyed")
        try:
            pygame.quit()
        except Exception as e:
            self.logger.debug(
                f"Cannot quit pygame normally, force quitting. Error: {e}")
        self.logger.debug("Game ended")

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        """
        Convert data from source to agent

        Returns:
            sensors_data: sensor data for agent
            new_vehicle: the current player's vehicle state

        """
        sensor_data: SensorsData = \
            self.carla_bridge.convert_sensor_data_from_source_to_agent(
                {
                    "front_rgb": None if self.world.front_rgb_sensor_data is None
                    else self.world.front_rgb_sensor_data,
                    "rear_rgb": None if self.world.rear_rgb_sensor_data is None
                    else self.world.rear_rgb_sensor_data,
                    "front_depth":
                        None if self.world.front_depth_sensor_data is None else
                        self.world.front_depth_sensor_data,
                    "imu": self.world.imu_sensor
                }
            )
        new_vehicle = self.carla_bridge.convert_vehicle_from_source_to_agent(
            self.world.player)
        return sensor_data, new_vehicle

    def execute_npcs_step(self):
        # TODO this can be parallelized
        try:
            for agent, actor in self.npc_agents.items():
                new_vehicle = self.carla_bridge.convert_vehicle_from_source_to_agent(actor)
                curr_control: VehicleControl = agent.run_in_series(sensors_data=SensorsData(), vehicle=new_vehicle)
                carla_control = self.carla_bridge.convert_control_from_agent_to_source(curr_control)
                actor.apply_control(carla_control)
        except Exception as e:
            self.logger.error(f"Failed to execute step for NPC. "
                              f"Error: {e}")

    def spawn_npcs(self):
        # parse npc file
        npc_config_file_path = Path(self.carla_settings.npc_config_file_path)
        assert npc_config_file_path.exists(), f"NPC file path {npc_config_file_path.as_posix()} does not exist"
        npc_configs = json.load(npc_config_file_path.open('r'))

        npc_configs: List[AgentConfig] = [AgentConfig.parse_obj(config) for config in npc_configs]

        self.world.spawn_npcs(npc_configs)
        self.npc_agents = {
            self.npc_agent_class(vehicle=actor, agent_settings=npc_config): actor for actor, npc_config in
            self.world.npcs_mapping.values()
        }
