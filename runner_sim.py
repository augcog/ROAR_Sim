import logging
from pathlib import Path
import numpy as np
import os
import warnings
from ROAR.roar_autonomous_system.configurations.configuration import Configuration
from ROAR.carla_client.carla_runner import CarlaRunner
from ROAR.roar_autonomous_system.agent_module.point_cloud_map_recording_agent import PointCloudMapRecordingAgent
from ROAR.roar_autonomous_system.agent_module.visualizer_demo_agent import VisualizerDemoAgent
def main():
    config = Configuration.parse_file(
        Path(os.getcwd()) / "configurations" / "config.json"
    )

    carla_runner = CarlaRunner(carla_settings=config.carla_config,
                               agent_settings=config.agent_config)
    try:
        my_vehicle = carla_runner.set_carla_world()

        # agent = PurePursuitAgent(vehicle=my_vehicle, agent_settings=config.agent_config)
        agent = PointCloudMapRecordingAgent(vehicle=my_vehicle, agent_settings=config.agent_config)
        # agent = VisualizerDemoAgent(vehicle=my_vehicle, agent_settings=config.agent_config)
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
    except Exception as e:
        logging.error(f"Something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)

    main()



