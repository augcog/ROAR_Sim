from pydantic import BaseModel, Field
from carla_client.util.utilities import CarlaWeathers, CarlaWeather, CarlaCarColors, CarlaCarColor
import os
from pathlib import Path

class CarlaSettings(BaseModel):
    host: str = Field(default="127.0.0.1", title="Host IP", description="The IP Address of the Carla Server")
    port: int = Field(default=2000, title="Host Port", description="The Port number of the Carla Server")
    enable_autopilot: bool = Field(default=False, title="Enable Antopilot", description="Enable Antopilot")
    width: int = Field(default=1280, title="Width of Window")
    height: int = Field(default=720, title="Height of Window")
    carla_vehicle_blueprint_filter: str = Field(default='vehicle.tesla.model3',
                                                title="Carla Vehicle Blueprint",
                                                description="For more detail, see https://carla.readthedocs.io/en/0.9.9/bp_library/")
    role_name: str = Field(default="hero", title="Name",
                           description="Your name in Carla, can be used to correctly identify you later")
    gamma: float = Field(default=2.2, title="Gamma Correction", description="Gamma Correction of the camera")
    car_color: CarlaCarColor = Field(default=CarlaCarColors.RED, title="Vehicle color",
                                     description="Your Vehicle Color")
    carla_weather: CarlaWeather = Field(default=CarlaWeathers.SUNNY, title="Carla Weather",
                                        description="Weather Setting")
    spawn_point_id: int = Field(default=1, title="Spaning Location ID", description="Spanning Location ID")
    data_file_path: str = Field(default=(Path(os.getcwd()) / "data" / "easy_map_waypoints.txt").as_posix())

