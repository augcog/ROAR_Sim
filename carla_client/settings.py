from pathlib import Path
import os, sys

carla_client_folder_path = Path(os.getcwd()) / "carla_client"
if sys.platform == 'darwin':
    assert False, "MacOS is currently not supported"
carla_client_egg_file_name = 'carla-0.9.9-py3.7-win-amd64.egg' if sys.platform == "win32" else \
    "carla-0.9.9-py3.6-linux-x86_64.egg"
carla_client_egg_file_path = carla_client_folder_path / carla_client_egg_file_name
if not carla_client_egg_file_path.is_file():
    raise FileNotFoundError("Please make sure carla client distribution is installed under the carla_client directory")
sys.path.append(carla_client_egg_file_path.as_posix())

from pydantic import BaseModel, Field
from carla_client.util.utilities import CarlaWeathers, CarlaWeather, CarlaCarColors, CarlaCarColor
from pathlib import Path
from typing import Optional
from roar_autonomous_system.utilities_module.camera_models import Camera
from roar_autonomous_system.utilities_module.data_structures_models import Location, Rotation


class CarlaSettings(BaseModel):
    # carla server setting
    host: str = Field(default="127.0.0.1", title="Host IP", description="The IP Address of the Carla Server")
    port: int = Field(default=2000, title="Host Port", description="The Port number of the Carla Server")

    # PyGame display setting
    width: int = Field(default=1280, title="Width of Display")
    height: int = Field(default=720, title="Height of Display")

    # carla world settings
    carla_weather: CarlaWeather = Field(default=CarlaWeathers.SUNNY, title="Carla Weather",
                                        description="Weather Setting")

    # carla vehicle setting
    carla_vehicle_blueprint_filter: str = Field(default='vehicle.tesla.model3',
                                                title="Carla Vehicle Blueprint",
                                                description="For more detail, see "
                                                            "https://carla.readthedocs.io/en/0.9.9/bp_library/")
    role_name: str = Field(default="hero", title="Name",
                           description="Your name in Carla, can be used to correctly identify you later")
    car_color: CarlaCarColor = Field(default=CarlaCarColors.RED, title="Vehicle color",
                                     description="Your Vehicle Color")

    # main camera setting (the third person camera)
    gamma: float = Field(default=2.2, title="Gamma Correction", description="Gamma Correction of the camera")

    # ROAR sensors settings
    front_depth_cam: Camera = Field(default=Camera(fov=70,
                                                   location=Location(x=1.6, y=0, z=1.7),
                                                   rotation=Rotation(pitch=0, yaw=0, roll=0),
                                                   image_size_x=800,
                                                   image_size_y=600),
                                    title="Front Depth Camera")
    front_rgb_cam: Camera = Field(default=Camera(fov=70,
                                                 location=Location(x=1.6, y=0, z=1.7),
                                                 rotation=Rotation(pitch=0, yaw=0, roll=0),
                                                 image_size_x=800,
                                                 image_size_y=600),
                                  title="Front RGB Camera")
    rear_rgb_cam: Camera = Field(default=Camera(fov=145,
                                                location=Location(x=-1.5, y=0.0, z=1.4),
                                                rotation=Rotation(pitch=0.0, yaw=180, roll=0.0),
                                                image_size_x=800,
                                                image_size_y=600),
                                 title="Rear RGB Camera")
    # data path
    waypoint_file_path: str = Field(default=(Path(os.getcwd()) / "data" / "easy_map_waypoints.txt").as_posix())
    output_data_folder_path: str = Field(default=(Path(os.getcwd()) / "data" / "output"))
    # this file should exist in ./ROAR/client_client/carla-X-X-X-OS-X.egg.
    # Setting this path will overwrite the default that is hard coded
    carla_client_egg_file_path: Optional[str] = Field(default=None)

    # miscellaneous settings
    enable_autopilot: bool = Field(default=False, title="Enable Antopilot", description="Enable Antopilot")
    spawn_point_id: int = Field(default=1, title="Spaning Location ID", description="Spanning Location ID")
    show_sensors_data: bool = Field(default=False)
    graph_post_modem_data: bool = Field(default=False)
    save_sensor_data: bool = Field(default=False)
    print_keyboard_hint: bool = Field(default=False)
