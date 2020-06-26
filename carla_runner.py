import os
import sys
from pathlib import Path
import logging
import pygame
from roar_autonomous_system.agents.path_following_agent import PathFollowingAgent


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
                                       target_speed=40
                                       )
        logger.debug("Initiating Game")
        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            should_continue, carla_control = controller.parse_events(client=client, world=world, clock=clock)
            if not should_continue:
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            if settings.enable_autopilot:
                # this is cheating here
                agent.vehicle = carla_bridge.convert_vehicle_from_source_to_agent(world.player)
                agent_control = agent.run_step()
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

    settings = CarlaSettings()
    settings.enable_autopilot = True
    try:
        game_loop(settings, logger)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == "__main__":
    main()
