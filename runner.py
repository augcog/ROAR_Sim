import logging
from pathlib import Path

import numpy as np

from ROAR_simulation.carla_client.carla_runner import CarlaRunner
from ROAR_simulation.carla_client.settings import CarlaSettings
from ROAR_simulation.roar_autonomous_system.agent_module.waypoint_following_agent import (
    WaypointFollowingAgent,
)


def main():
    log_level = logging.DEBUG
    logging.basicConfig(
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=log_level,
    )
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    settings = CarlaSettings()
    np.set_printoptions(suppress=True)

    try:
        carla_runner = CarlaRunner(carla_settings=settings)
        my_vehicle = carla_runner.set_carla_world()

        agent = WaypointFollowingAgent(
            vehicle=my_vehicle,
            should_save_sensor_data=settings.save_sensor_data,
            output_folder_path=Path(settings.output_data_folder_path),
            route_file_path=Path(settings.waypoint_file_path),
            target_speed=80,
            front_depth_camera=settings.front_depth_cam,
            front_rgb_camera=settings.front_rgb_cam,
            rear_rgb_camera=settings.rear_rgb_cam,
        )
        carla_runner.start_game_loop(agent=agent, use_manual_control=True)
    except Exception as e:
        print(f"ERROR: Something bad happened. Safely exiting. "
              f"You are going to want to restart the server. Error:{e}")


if __name__ == "__main__":
    main()
