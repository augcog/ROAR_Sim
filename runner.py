from ROAR_simulation.carla_client.carla_settings import CarlaConfig
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings \
    import \
    AgentConfig
from ROAR_simulation.carla_client.carla_runner import CarlaRunner
import logging
from ROAR_simulation.roar_autonomous_system.agent_module\
    .semantic_segmentation_agent import \
    SemanticSegmentationAgent
from ROAR_simulation.roar_autonomous_system.agent_module.purpursuit_agent \
    import PurePursuitAgent
from ROAR_simulation.roar_autonomous_system.agent_module.mpc_agent import MPCAgent
from ROAR_simulation.roar_autonomous_system.agent_module.pid_agent import PIDAgent
from pathlib import Path
import numpy as np
import os
import warnings


def main():
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)

    try:
        carla_settings = CarlaConfig()
        agent_settings = AgentConfig.parse_file(
            Path(os.getcwd()) / "ROAR_simulation" / "agent_config.json")
        carla_runner = CarlaRunner(carla_settings=carla_settings,
                                   agent_settings=agent_settings)
        my_vehicle = carla_runner.set_carla_world()
        # agent = PurePursuitAgent(vehicle=my_vehicle,
        #                          agent_settings=agent_settings)

        # agent = SemanticSegmentationAgent(
        #     vehicle=my_vehicle,
        #     agent_settings=agent_settings
        # )
        #
        agent = PIDAgent(
            vehicle=my_vehicle,
            agent_settings=agent_settings,
            target_speed=200
        )
        # agent = MPCAgent(
        #     vehicle=my_vehicle,
        #     agent_settings=agent_settings,
        #     target_speed=60
        # )
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
    except Exception as e:
        logging.error(f"{e}. Need to restart Server")


if __name__ == '__main__':
    main()
