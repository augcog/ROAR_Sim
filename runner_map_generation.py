import logging
from pathlib import Path
import numpy as np
import os
import warnings
from ROAR.roar_autonomous_system.configurations.configuration import Configuration
from ROAR.carla_client.carla_runner import CarlaRunner
from ROAR.roar_autonomous_system.agent_module.map_generating_agent_v3 import MapGeneratingAgentV3
from ROAR.roar_autonomous_system.agent_module.point_cloud_map_recording_agent import PointCloudMapRecordingAgent

def main():
    config = Configuration.parse_file(
        Path(os.getcwd()) / "configurations" / "config.json"
    )

    carla_runner = CarlaRunner(carla_settings=config.carla_config,
                               agent_settings=config.agent_config)
    try:
        my_vehicle = carla_runner.set_carla_world()
        agent = PointCloudMapRecordingAgent(vehicle=my_vehicle, agent_settings=config.agent_config)
        # agent = MapGeneratingAgentV3(agent_settings=config.agent_config, vehicle=my_vehicle)
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
    except Exception as e:
        print("Ending abnormally: ", e)
        carla_runner.on_finish()
        logging.error(f"Hint: Might be a good idea to restart Server. ")


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)

    main()