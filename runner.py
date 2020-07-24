from ROAR_simulation.carla_client.settings import CarlaSettings
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings \
    import \
    AgentSettings
from ROAR_simulation.carla_client.carla_runner import CarlaRunner
import logging
from ROAR_simulation.roar_autonomous_system.agent_module\
    .waypoint_following_agent import \
    WaypointFollowingAgent
from ROAR_simulation.roar_autonomous_system.agent_module.purpursuit_agent \
    import \
    PurePursuitAgent
from ROAR_simulation.roar_autonomous_system.agent_module\
    .semantic_segmentation_agent import \
    SemanticSegmentationAgent
from pathlib import Path
import numpy as np


def main():
    log_level = logging.DEBUG
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=log_level)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    carla_settings = CarlaSettings()
    agent_settings = AgentSettings()
    np.set_printoptions(suppress=True)

    try:
        carla_runner = CarlaRunner(carla_settings=carla_settings,
                                   agent_settings=agent_settings)
        my_vehicle = carla_runner.set_carla_world()
        agent = PurePursuitAgent(vehicle=my_vehicle,
                                 agent_settings=agent_settings)

        # agent = SemanticSegmentationAgent(
        #     vehicle=my_vehicle,
        #     agent_settings=agent_settings
        # )
        # agent = WaypointFollowingAgent(vehicle=my_vehicle,
        #                                agent_settings=agent_settings,
        #                                target_speed=120)
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
    except Exception as e:
        print(f"ERROR: Something bad happened. Safely exiting. Error:{e}")


if __name__ == '__main__':
    main()
