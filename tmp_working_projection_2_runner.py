import os
import sys
from pathlib import Path
import logging
import pygame
from roar_autonomous_system.agents.path_following_agent import PathFollowingAgent
import cv2
from roar_autonomous_system.perception.ground_plane_detector import GroundPlaneDetector
from roar_autonomous_system.agents.gpd_agent import GPDAgent

"""
    The import order like this is very important! 
"""
carla_client_folder_path = Path(os.getcwd()) / "carla_client"

if sys.platform == 'darwin':
    assert False, "MacOS is currently not supported"
carla_client_egg_file_name = 'carla-0.9.9-py3.7-win-amd64.egg' if sys.platform == "win32" else \
    "carla-0.9.9-py3.6-linux-x86_64.egg"
carla_client_egg_file_path = carla_client_folder_path / carla_client_egg_file_name
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
from roar_autonomous_system.util.models import SensorData, Vehicle
from typing import Tuple
from carla_client.util.utilities import create_dir_if_not_exist
import numpy as np


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
        vehicle = carla_bridge.convert_vehicle_from_source_to_agent(world.player)
        if settings.enable_autopilot:
            agent = PathFollowingAgent(
                vehicle=vehicle,
                bridge=carla_bridge,
                route_file_path=Path(settings.waypoint_file_path)
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
                if sensor_data.front_rgb is not None and sensor_data.front_rgb.data is not None:
                    cv2.imshow('front_rgb_data', sensor_data.front_rgb.data)
                if sensor_data.front_depth is not None and sensor_data.front_depth.data is not None:
                    cv2.imshow('front_depth_data', sensor_data.front_depth.data)
                if sensor_data.rear_rgb is not None and sensor_data.rear_rgb.data is not None:
                    cv2.imshow('rear_rgb_data', sensor_data.rear_rgb.data)
                cv2.waitKey(1)
            if settings.save_sensor_data:
                if sensor_data.front_rgb is not None and sensor_data.front_rgb.data is not None:
                    cv2.imwrite((Path(
                        settings.output_data_folder_path) / "front_rgb" / f"front_rgb-{world.time_counter}.png").as_posix(),
                                sensor_data.front_rgb.data)
                if sensor_data.front_depth is not None and sensor_data.front_depth.data is not None:
                    cv2.imwrite((Path(
                        settings.output_data_folder_path) / "front_depth" / f"depth-{world.time_counter}.png").as_posix(),
                                sensor_data.front_depth.data)
                if sensor_data.rear_rgb is not None and sensor_data.rear_rgb.data is not None:
                    cv2.imwrite((Path(
                        settings.output_data_folder_path) / "rear_rgb" / f"rear_rgb-{world.time_counter}.png").as_posix(),
                                sensor_data.rear_rgb.data)

            if settings.enable_autopilot:
                agent_control = agent.run_step(vehicle=new_vehicle, sensor_data=sensor_data)
                # carla_control = carla_bridge.convert_control_from_agent_to_source(agent_control)
            world.player.apply_control(carla_control)

            waypoint_0 = [8.670342445373535, 50.49717712402344, -0.009518756531178951, 1]
            waypoint_1 = [8.63516616821289, 43.11967849731445, -0.005509738810360432, 1]
            waypoint_2 = [8.616731643676758, 39.23454284667969, -0.005410938058048487, 1]
            waypoint_3 = [8.542390823364258, 23.610092163085938, -0.005435962695628405, 1]

            img_coord0 = calculate_img_pos(waypoint_0, world.player.get_transform(),
                                           world.front_rgb_sensor.get_transform())
            img_coord1 = calculate_img_pos(waypoint_1, world.player.get_transform(),
                                           world.front_rgb_sensor.get_transform())
            img_coord2 = calculate_img_pos(waypoint_2, world.player.get_transform(),
                                           world.front_rgb_sensor.get_transform())
            img_coord3 = calculate_img_pos(waypoint_3, world.player.get_transform(),
                                           world.front_rgb_sensor.get_transform())

            img_coord0 = img_coord0.astype(np.int64)
            img_coord1 = img_coord1.astype(np.int64)
            img_coord2 = img_coord2.astype(np.int64)
            img_coord3 = img_coord3.astype(np.int64)
            img = sensor_data.front_rgb.data.copy()
            # img[480:480 + 4, 167:167 + 4] = [125, 125, 125]
            img[415:415 + 4, 400:400 + 4] = [255, 0, 0]
            img[379:379 + 4, 400:400 + 4] = [0, 255, 0]
            img[336:336 + 4, 400:400 + 4] = [0, 0, 255]
            print(img_coord0.T, "|", img_coord1.T, "|", img_coord2.T, "|", img_coord3.T)
            cv2.imshow("test", img)
            cv2.waitKey(1)

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


def calculate_img_pos(waypoint, curr_transform, sensor_transform):
    location = curr_transform.location
    rotation = curr_transform.rotation
    intrinsics = np.identity(3)
    intrinsics[0, 2] = 800 / 2.0
    intrinsics[1, 2] = 600 / 2.0
    intrinsics[0, 0] = intrinsics[1, 1] = 800 / (2.0 * np.tan(70 * np.pi / 360.0))

    # print(sensor_transform)
    # sensor_world_matrix = get_matrix(sensor_transform)
    # world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
    cam_veh_matrix = calculate_extrinsics_from_euler(pitch=0, yaw=0, roll=0, tx=1.6, ty=0, tz=1.7)
    veh_world_matrix = get_matrix(curr_transform)
    world_sensor_matrix = np.linalg.inv(cam_veh_matrix) @ np.linalg.inv(veh_world_matrix)
    cords_x_y_z = world_sensor_matrix @ np.array(waypoint)
    tmp = cords_x_y_z.T
    cords_y_minus_z_x = np.concatenate([tmp[1], -tmp[2], tmp[0]])
    distance = np.linalg.norm(np.array([waypoint[0], waypoint[1], waypoint[2]]) -
                              np.array([location.x, location.y, location.z]))
    p2d = intrinsics @ cords_y_minus_z_x
    temp = p2d.T
    camera = np.concatenate([temp[:, 0] / temp[:, 2], temp[:, 1] / temp[:, 2], temp[:, 2]])
    # print(np.shape(camera), "|", camera.T)
    return camera

def calculate_extrinsics_from_euler(pitch, yaw, roll, tx, ty, tz):
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.identity(4)
    matrix[0, 3] = tx
    matrix[1, 3] = ty
    matrix[2, 3] = tz
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix

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
    try:
        game_loop(settings, logger)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


def get_matrix(transform):
    """
    Creates matrix from carla transform.
    """

    rotation = transform.rotation
    location = transform.location
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix


if __name__ == "__main__":
    main()
