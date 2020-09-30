from ROAR.ROAR_Jetson.jetson_runner import JetsonRunner
from ROAR.roar_autonomous_system.agent_module.floodfill_based_lane_follower import FloodfillBasedLaneFollower
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle
from ROAR.ROAR_Jetson.jetson_config import JetsonConfig
from ROAR.roar_autonomous_system.configurations.agent_settings import AgentConfig
from pathlib import Path
import logging
import warnings
import numpy as np
import os
import json


def main():
    try:
        config_file_path = Path(os.getcwd()) / "configurations" / "jetson_config.json"
        config_file: dict = json.load(config_file_path.open('r'))
        jetson_config = JetsonConfig.parse_obj(config_file["jetson_config"])
        agent_config: AgentConfig = AgentConfig.parse_obj(config_file["agent_config"])
        status = allow_dev_access(read_password(Path(jetson_config.jetson_sudo_password_file_path)))
        assert status is True, "Port not successfully opened"
        # agent = ForwardOnlyAgent(vehicle=Vehicle(), agent_settings=agent_config)
        agent = FloodfillBasedLaneFollower(vehicle=Vehicle(), agent_settings=agent_config)
        print("Agent initialized")
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
