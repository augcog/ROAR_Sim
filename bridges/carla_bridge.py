import carla
from carla import ColorConverter as cc
from bridges.bridge import Bridge
from typing import Dict, Tuple, Union, Any
from roar_autonomous_system.util.models import SensorData, Control, RGBData, DepthData, IMUData, Vector3D, Vehicle, \
    Transform, Location, Rotation
import numpy as np
import math


class CarlaBridge(Bridge):

    def convert_location_from_source_to_agent(self, source: carla.Location) -> Location:
        return Location(x=source.x, y=source.y, z=source.z)

    def convert_rotation_from_source_to_agent(self, source: carla.Rotation) -> Rotation:
        return Rotation(pitch=source.pitch, yaw=source.yaw, roll=source.roll)

    def convert_transform_from_source_to_agent(self, source: carla.Transform) -> Transform:
        return Transform(
            location=self.convert_location_from_source_to_agent(source=source.location),
            rotation=self.convert_rotation_from_source_to_agent(source=source.rotation))

    def convert_control_from_source_to_agent(self, source: carla.VehicleControl) -> Control:
        return Control(throttle=-1 * source.throttle if source.reverse else source.throttle,
                       steering=source.steer)

    def convert_rgb_from_source_to_agent(self, source: carla.Image) -> RGBData:
        source.convert(cc.Raw)
        array = np.frombuffer(source.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (source.height, source.width, 4))  # RGBA format
        array = array[:, :, :3]  # take only RGB
        array = array[:, :, ::-1]  # BGR
        return array.swapaxes(0, 1)

    def convert_depth_from_source_to_agent(self, source: carla.Image) -> DepthData:
        source.convert(cc.LogarithmicDepth)
        array = np.frombuffer(source.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (source.height, source.width, 4))  # RGBA format
        array = array[:, :, :3]  # take only RGB
        array = array[:, :, ::-1]  # BGR
        return array.swapaxes(0, 1)

    def convert_vector3d_from_source_to_agent(self, source: carla.Vector3D) -> Vector3D:
        return Vector3D(x=source.x, y=source.y, z=source.z)

    def convert_imu_from_source_to_agent(self, source: carla.IMUMeasurement) -> IMUData:
        return IMUData(accelerometer=self.convert_vector3d_from_source_to_agent(source=source.accelerometer),
                       gyroscope=self.convert_vector3d_from_source_to_agent(source=source.gyroscope))

    def convert_sensor_data_from_source_to_agent(self, source: dict) -> SensorData:
        return SensorData(rgb=self.convert_rgb_from_source_to_agent(source=source.get("rgb", None)),
                          depth=self.convert_depth_from_source_to_agent(source=source.get("depth", None)),
                          imu_data=self.convert_imu_from_source_to_agent(source=source.get("imu", None)))

    def convert_vehicle_from_source_to_agent(self, source: carla.Vehicle) -> Vehicle:
        control: Control = self.convert_control_from_source_to_agent(source.get_control())
        transform: Transform = self.convert_transform_from_source_to_agent(source.get_transform())
        velocity: Vector3D = self.convert_vector3d_from_source_to_agent(source.get_velocity())
        return Vehicle(velocity=velocity, transform=transform, control=control)

    def convert_control_from_agent_to_source(self, control: Control) -> carla.VehicleControl:
        return carla.VehicleControl(throttle=abs(control.throttle),
                                    steer=control.steering,
                                    brake=0,
                                    hand_brake=False,
                                    reverse=True if control.throttle < 0 else False,
                                    manual_gear_shift=False,
                                    gear=1)

    def convert_vector3d_from_agent_to_source(self, vector3d: Vector3D) -> carla.Vector3D:
        return carla.Vector3D(x=vector3d.x, y=vector3d.y, z=vector3d.z)

    def convert_vehicle_from_agent_to_source(self, vehicle: Vehicle) -> Any:
        control: Control = self.convert_control_from_agent_to_source(control=vehicle.control)
        velocity: Vector3D = self.convert_vector3d_from_agent_to_source(vector3d=vehicle.velocity)
        transform: Transform = self.convert_transform_from_agent_to_source(transform=vehicle.transform)
        return Vehicle(velocity=velocity,
                       transform=transform,
                       control=control)
