from ROAR.ROAR_Jetson.jetson_runner import JetsonRunner
from ROAR.roar_autonomous_system.agent_module.forward_only_agent import ForwardOnlyAgent
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR.roar_autonomous_system.configurations.configuration import Configuration
from pathlib import Path
import logging
import warnings
import numpy as np
import os
import json


def main():
    try:

        config = Configuration.parse_file(
            Path(os.getcwd()) / "configurations" / "jetson_config.json"
        )
        status = allow_dev_access(read_password(Path(config.jetson_config.jetson_sudo_password_file_path)))
        assert status is True, "Port not successfully opened"

        agent = ForwardOnlyAgent(vehicle=Vehicle(), agent_settings=config.agent_config)
        jetson_runner = JetsonRunner(agent=agent, jetson_config=config.jetson_config)
        jetson_runner.start_game_loop(use_manual_control=True)
    except Exception as e:
        print(f"Something bad happened {e}")


def read_password(jetson_sudo_setup_f_path: Path):
    d = json.load(fp=jetson_sudo_setup_f_path.open("r"))
    return d["sudo_password"]

def allow_dev_access(pwd):
    command = 'chmod 777 /dev/ttyACM0'
    p = os.system(f'echo {pwd}|sudo -S {command}')
    return True if p == 0 else False

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)
    main()
