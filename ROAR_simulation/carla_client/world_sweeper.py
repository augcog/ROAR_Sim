""""
WARNING: running this file will wipe ALL actors in the world!!!
"""
from pathlib import Path
import os, sys

carla_client_folder_path = Path(os.getcwd()) / "carla_client"
carla_client_egg_file_path = (
    carla_client_folder_path / "carla-0.9.9-py3.7-win-amd64.egg"
)
if not carla_client_egg_file_path.is_file():
    raise FileNotFoundError(
        "Please make sure carla client distribution is installed under the carla_client directory"
    )
sys.path.append(carla_client_egg_file_path.as_posix())
sys.path.append(carla_client_folder_path.as_posix())
from settings import CarlaSettings
import carla


def wipe_all_actors():
    settings = CarlaSettings()
    client = carla.Client(settings.host, settings.port)
    all_actors = client.get_world().get_actors()
    for actor in all_actors:
        actor.destroy()


if __name__ == "__main__":
    wipe_all_actors()
