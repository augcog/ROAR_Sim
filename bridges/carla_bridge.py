import carla
from carla import ColorConverter as cc
from bridges.bridge import Bridge
from typing import Dict, Tuple, Union
from roar_autonomous_system.util.models import SensorData, Control, RGBData, DepthData, IMUData, Vector3D, Vehicle, \
    Transform, Location, Rotation
import numpy as np
import math


class CarlaBridge(Bridge):
    def convert_control_data_from_source_to_agent(self, source: carla.VehicleControl) -> Control:
        return Control(throttle=source.throttle,
                       steering=source.steer)

    def convert_control_data_from_agent_to_source(self, control: Control) -> carla.VehicleControl:
        return carla.VehicleControl(throttle=abs(control.throttle),
                                    steer=control.steering,
                                    brake=0,
                                    handbrake=False,
                                    reverse=True if control.throttle < 1 else False,
                                    manual_gear_shift=False,
                                    gear=1)

    def convert_sensor_data_from_source_to_agent(self,
                                                 source: Dict[str,
                                                              Union[carla.Image,
                                                                    Tuple[float, float, float]]]) -> SensorData:
        raw_image: carla.Image = source["rgb"]
        rgb = self._convert_carla_image_to_agent_image(image=raw_image, converter=cc.Raw)
        depth = self._convert_carla_image_to_agent_image(image=raw_image, converter=cc.Depth)
        accel = source["accel"]
        gyro = source["gyro"]
        img_data = IMUData(accelerometer=Vector3D(x=accel[0], y=accel[0], z=accel[0]),
                           gyroscope=Vector3D(x=gyro[0], y=gyro[0], z=gyro[0]))
        return SensorData(rgb=rgb, depth=depth, imu_data=img_data)

    @staticmethod
    def _convert_carla_image_to_agent_image(image: carla.Image, converter):
        image.convert(converter)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
        array = array[:, :, :3]  # take only RGB
        array = array[:, :, ::-1]  # BGR
        return array.swapaxes(0, 1)

    def convert_agent_control_to_carla_control(self, agent_control: Control) -> carla.VehicleControl:
        return carla.VehicleControl(throttle=agent_control.throttle, steer=agent_control.steering)

    def convert_source_vehicle_to_agent_vehicle(self, source_vehicle: carla.Vehicle) -> Vehicle:
        control: Control = self.convert_control_data_from_source_to_agent(source_vehicle.get_control())
        transform: Transform = self.convert_carla_transform_to_agent_transform(source_vehicle.get_transform())
        velocity: Vector3D = self.convert_carla_velocity_to_agent_velocity(source_vehicle.get_velocity())
        return Vehicle(velocity=velocity, transform=transform, control=control)

    def convert_carla_transform_to_agent_transform(self, carla_transform: carla.Transform) -> Transform:
        return Transform(
            location=self.convert_carla_location_to_agent_location(carla_location=carla_transform.location),
            rotation=self.convert_carla_rotation_to_agent_rotation(carla_rotation=carla_transform.rotation))

    @staticmethod
    def convert_carla_location_to_agent_location(carla_location: carla.Location) -> Location:
        return Location(x=carla_location.x, y=carla_location.y, z=carla_location.z)

    @staticmethod
    def convert_carla_rotation_to_agent_rotation(carla_rotation: carla.Rotation) -> Rotation:
        return Rotation(pitch=carla_rotation.pitch, yaw=carla_rotation.yaw, roll=carla_rotation.roll)

    @staticmethod
    def convert_carla_velocity_to_agent_velocity(carla_velocity: carla.Vector3D):
        return Vector3D(x=carla_velocity.x, y=carla_velocity.y, z=carla_velocity.z)

