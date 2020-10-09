from ROAR.roar_autonomous_system.agent_module.agent import Agent
import logging, warnings
import numpy as np
from ROAR.roar_autonomous_system.configurations.configuration import Configuration
from pathlib import Path
import os
from ROAR.roar_autonomous_system.agent_module.pure_pursuit_agent \
    import PurePursuitAgent
from ROAR.carla_client.carla_competition_runner import CarlaRunner


def run(agent_class, config_fpath) -> float:
    """
    Run the agent along the track and produce a score based on certain metrics
    Args:
        agent_class:
        config_fpath:

    Returns:
        float between 0 - 1 representing scores
    """
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.INFO)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)
    config = Configuration.parse_file(config_fpath)

    carla_runner = CarlaRunner(carla_settings=config.carla_config,
                               agent_settings=config.agent_config, max_collision=3)
    try:
        my_vehicle = carla_runner.set_carla_world()
        agent = agent_class(vehicle=my_vehicle, agent_settings=config.agent_config)
        score = carla_runner.start_game_loop(agent=agent, use_manual_control=True)
        return score
    except Exception as e:
        print(f"something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")
        return 0


def main():
    agent_class = PurePursuitAgent
    num_trials = 2
    total_score = 0
    for i in range(num_trials):
        score = run(agent_class=agent_class, config_fpath=Path(os.getcwd()) / "configurations" / "configuration.json")
        print(f"This run's score is {score}")
        total_score += score
    print(f"Your Average is {total_score / num_trials}")


if __name__ == "__main__":
    main()
