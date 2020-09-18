from ROAR_Jetson.jetson_runner import JetsonRunner
from ROAR_simulation.roar_autonomous_system.agent_module.forward_only_agent import ForwardOnlyAgent
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Vector3D
from ROAR_simulation.roar_autonomous_system.configurations.configuration import Configuration
from pathlib import Path
import os
import logging
import warnings
import numpy as np


def main():

    try:
        config = Configuration.parse_file(
            Path(os.getcwd()) / "configurations" / "config.json"
        )
        agent = ForwardOnlyAgent(vehicle=Vehicle(), agent_settings=config.agent_config)
        jetson_runner = JetsonRunner(agent=agent)
        jetson_runner.start_game_loop(use_manual_control=True)
    except Exception as e:
        print(f"Something bad happened {e}")


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)
    main()
