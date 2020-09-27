from ROAR.ROAR_Jetson.jetson_runner import JetsonRunner
from ROAR.roar_autonomous_system.agent_module.forward_only_agent import ForwardOnlyAgent
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR.roar_autonomous_system.configurations.agent_settings import AgentConfig
from ROAR.ROAR_Jetson.jetson_config import JetsonConfig
from pathlib import Path
import logging
import warnings
import numpy as np
import os
import json


def main():
    try:
        config_file_path = Path(os.getcwd()) / "configurations" / "jetson_config.json"
        agent_config = AgentConfig.parse_file(config_file_path)
        jetson_config = JetsonConfig.parse_file(config_file_path)
        status = allow_dev_access(read_password(Path(jetson_config.jetson_sudo_password_file_path)))
        assert status is True, "Port not successfully opened"

        agent = ForwardOnlyAgent(vehicle=Vehicle(), agent_settings=agent_config)
        jetson_runner = JetsonRunner(agent=agent, jetson_config=jetson_config)
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
