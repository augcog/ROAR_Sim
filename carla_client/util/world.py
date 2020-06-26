import carla
import logging
import random
import sys
from settings import CarlaSettings
from util.hud import HUD
from util.utilities import CarlaCarColor, CarlaCarColors, get_actor_display_name
from util.sensors import CollisionSensor, GnssSensor,LaneInvasionSensor,IMUSensor,RadarSensor
from util.camera_manager import CameraManager

class World(object):
    def __init__(self, carla_world:carla.World, hud:HUD, settings:CarlaSettings):
        self.logger = logging.getLogger(__name__)
        self.world: carla.World = carla_world
        self.actor_role_name = settings.role_name
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self._spawn_point_id = settings.spawn_point_id
        self._actor_filter = settings.carla_vehicle_blueprint_filter
        self._car_color = settings.car_color # todo parse arg here
        self._gamma = settings.gamma
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self.recording_enabled = False
        self.recording_start = 0
        # set weather
        self.logger.debug("Setting Weather")
        self.set_weather(settings.carla_weather.to_carla_weather_params())

        # set player
        self.logger.debug("Setting Player")
        self.set_player(actor_filter=self._actor_filter,
                        player_role_name=self.actor_role_name,
                        color=self._car_color,
                        spawn_point_id=self._spawn_point_id)
        # set camera
        self.logger.debug("Setting Camera")
        self.set_camera()

        # set sensor
        self.logger.debug("Setting Sensor")
        self.set_sensor()

        self.world.on_tick(hud.on_world_tick)
        self.logger.debug("World Initialized")

    def set_player(self, actor_filter: str = "vehicle.tesla.model3", player_role_name: str = "hero",
                   color: CarlaCarColor = CarlaCarColors.GREY, spawn_point_id: int = random.choice(list(range(8)))):
        blueprint = self.world.get_blueprint_library().find(actor_filter)
        blueprint.set_attribute('role_name', player_role_name)
        if blueprint.has_attribute('color'):
            blueprint.set_attribute('color', color.to_string())
        if blueprint.has_attribute('is_invincible'):
            self.logger.debug("TESLA IS INVINCIBLE")
            blueprint.set_attribute('is_invincible', 'true')
        try:
            self.player = self.world.try_spawn_actor(blueprint, self.map.get_spawn_points()[spawn_point_id])
        except Exception as e:
            raise ValueError(f"Cannot spawn actor at ID [{spawn_point_id}]. Error: {e}")

    def set_camera(self, cam_index: int = 0, cam_pos_index: int = 0):
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def set_sensor(self):
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        self.logger.debug(f"destroying all actors belonging to [{self.actor_role_name}] in this world")
        if self.radar_sensor is not None:
            self.toggle_radar()
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def set_weather(self, new_weather: carla.WeatherParameters):
        self.world.weather = new_weather
