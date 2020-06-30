import os
import sys
from pathlib import Path
import logging
import pygame
from roar_autonomous_system.agents.path_following_agent import PathFollowingAgent
import cv2
import numpy as np

"""
    The import order like this is very important! 
"""
carla_client_folder_path = Path(os.getcwd()) / "carla_client"
carla_client_egg_file_path = carla_client_folder_path / 'carla-0.9.9-py3.7-win-amd64.egg'
if not carla_client_egg_file_path.is_file():
    raise FileNotFoundError("Please make sure carla client distribution is installed under the carla_client directory")
sys.path.append(carla_client_egg_file_path.as_posix())
sys.path.append(carla_client_folder_path.as_posix())
from carla_client.settings import CarlaSettings
from carla_client.util.keyboard_control import KeyboardControl
from carla_client.util.hud import HUD
from carla_client.util.world import World
from bridges.carla_bridge import CarlaBridge
import carla
from roar_autonomous_system.util.models import RGBData, DepthData, SensorData, Vehicle
from typing import Union, Tuple
from carla_client.util.utilities import create_dir_if_not_exist


def convert_data(world, carla_bridge) -> Tuple[SensorData, Vehicle]:
    sensor_data: SensorData = carla_bridge.convert_sensor_data_from_source_to_agent(
        {
            "front_rgb": None if world.front_rgb_sensor_data is None else world.front_rgb_sensor_data,
            "rear_rgb": None if world.rear_rgb_sensor_data is None else world.rear_rgb_sensor_data,
            "front_depth": None if world.front_depth_sensor_data is None else world.front_depth_sensor_data,
            "imu": world.imu_sensor
        }
    )
    new_vehicle = carla_bridge.convert_vehicle_from_source_to_agent(world.player)
    return sensor_data, new_vehicle


def game_loop(settings: CarlaSettings, logger: logging.Logger):
    pygame.init()
    pygame.font.init()
    world = None
    client = None
    try:
        carla_bridge = CarlaBridge()
        agent = None
        logger.debug(f"Connecting to {settings.host}:{settings.port}")
        client = carla.Client(settings.host, settings.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode((settings.width, settings.height), pygame.HWSURFACE | pygame.DOUBLEBUF)

        logger.debug(f"Setting HUD")
        hud = HUD(settings.width, settings.height)

        logger.debug(f"Setting up World")
        world = World(client.get_world(), hud, settings)

        logger.debug(f"Connecting Keyboard Controls")
        controller = KeyboardControl(world, settings.enable_autopilot, print_instruction=False)
        if settings.enable_autopilot:
            agent = PathFollowingAgent(vehicle=carla_bridge.convert_vehicle_from_source_to_agent(world.player),
                                       route_file_path=Path(settings.data_file_path),
                                       bridge=carla_bridge,
                                       visualize_occupancy_map=False
                                       )
        logger.debug("Initiating Game")
        clock = pygame.time.Clock()
        while True:
            # make sure the program does not run above 40 frames per second
            # this allow proper synchrony between server and client
            clock.tick_busy_loop(60)
            should_continue, carla_control = controller.parse_events(client=client, world=world, clock=clock)
            if not should_continue:
                break
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            sensor_data, new_vehicle = convert_data(world, carla_bridge)
            if settings.show_sensors_data:
                if world.front_rgb_sensor_data is not None:
                    cv2.imshow('front_rgb_data', sensor_data.front_rgb.data)
                if world.front_depth_sensor_data is not None:
                    cv2.imshow('front_depth_data', sensor_data.front_depth.data)
                if world.rear_rgb_sensor_data is not None:
                    cv2.imshow('rear_rgb_data', sensor_data.rear_rgb.data)

            if settings.save_sensor_data:
                if sensor_data.front_rgb is not None:
                    cv2.imwrite((Path(
                        settings.output_data_folder_path) / "front_rgb" / f"front_rgb-{world.time_counter}.png").as_posix(),
                                sensor_data.front_rgb.data)
                if sensor_data.front_depth is not None:
                    np.save(Path(settings.output_data_folder_path) / "front_depth" / f"depth-{world.time_counter}",
                            sensor_data.front_depth.data)
                    # cv2.imwrite((Path(settings.output_data_folder_path) / "front_depth" / f"depth-{world.time_counter}.png").as_posix(),
                    #             sensor_data.front_depth.data)
                if sensor_data.rear_rgb is not None:
                    cv2.imwrite((Path(
                        settings.output_data_folder_path) / "rear_rgb" / f"rear_rgb-{world.time_counter}.png").as_posix(),
                                sensor_data.rear_rgb.data)

            if settings.enable_autopilot:
                agent_control = agent.run_step(vehicle=new_vehicle, sensor_data=sensor_data)
                carla_control = carla_bridge.convert_control_from_agent_to_source(agent_control)
            world.player.apply_control(carla_control)


    except Exception as e:
        logger.error(f"Safely exiting due to error: {e}")

    finally:
        if world and world.recording_enabled:
            logger.debug("Stopping Recording")
            if client is not None:
                client.stop_recorder()

        logger.debug("Ending Game")
        if world is not None:
            world.destroy()
            logger.debug("All actors are destroyed")
        try:
            pygame.quit()
        except Exception as e:
            logger.debug(f"Cannot quit pygame normally, force quitting. Error: {e}")
        logger.debug("Game ended")
        exit(0)


def main():
    log_level = logging.DEBUG
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', level=log_level)
    logger = logging.getLogger(__name__)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    settings = CarlaSettings()
    settings.enable_autopilot = True
    settings.show_sensors_data = False
    settings.save_sensor_data = False
    settings.graph_post_modem_data = False

    if settings.save_sensor_data:
        create_dir_if_not_exist((Path(settings.output_data_folder_path) / "front_depth"))
        create_dir_if_not_exist((Path(settings.output_data_folder_path) / "front_rgb"))
        create_dir_if_not_exist((Path(settings.output_data_folder_path) / "rear_rgb"))
        create_dir_if_not_exist((Path(settings.output_data_folder_path) / "city_pallet"))
    try:
        game_loop(settings, logger)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == "__main__":
    main()
