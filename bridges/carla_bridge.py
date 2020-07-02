import carla
from carla import ColorConverter as cc
from carla_client.util.sensors import IMUSensor
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

    def convert_rgb_from_source_to_agent(self, source: carla.Image) -> Union[RGBData, None]:
        try:
            source.convert(cc.Raw)
            return RGBData(data=self._to_rgb_array(source))
        except:
            return None

    def convert_depth_from_source_to_agent(self, source: carla.Image) -> Union[DepthData, None]:
        try:
            source.convert(cc.LogarithmicDepth)
            return DepthData(data=self._to_depth_array(source))
        except:
            return None

    def convert_vector3d_from_source_to_agent(self, source: carla.Vector3D) -> Vector3D:
        return Vector3D(x=source.x, y=source.y, z=source.z)

    def convert_imu_from_source_to_agent(self, source:IMUSensor) -> IMUData:
        return IMUData(accelerometer=Vector3D(x=source.accelerometer[0],
                                              y=source.accelerometer[1],
                                              z=source.accelerometer[2]),
                       gyroscope=Vector3D(x=source.gyroscope[0],
                                          y=source.gyroscope[1],
                                          z=source.gyroscope[2]))

    def convert_sensor_data_from_source_to_agent(self, source: dict) -> SensorData:
        return SensorData(front_rgb=self.convert_rgb_from_source_to_agent(source=source.get("front_rgb", None)),
                          rear_rgb=self.convert_rgb_from_source_to_agent(source=source.get("rear_rgb", None)),
                          front_depth=self.convert_depth_from_source_to_agent(source=source.get("front_depth", None)),
                          imu_data=self.convert_imu_from_source_to_agent(source=source.get("imu", None)))

    def convert_vehicle_from_source_to_agent(self, source: carla.Vehicle) -> Vehicle:
        control: Control = self.convert_control_from_source_to_agent(source.get_control())
        # this is cheating here, vehicle does not know its own location
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

    def _to_bgra_array(self, image):
        """Convert a CARLA raw image to a BGRA numpy array."""
        if not isinstance(image, carla.Image):
            raise ValueError("Argument must be a carla.sensor.Image")
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        return array

    def _to_rgb_array(self, image):
        """Convert a CARLA raw image to a RGB numpy array."""
        array = self._to_bgra_array(image)
        # Convert BGRA to RGB.
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        return array

    def _to_depth_array(self, image):
        """
        Convert an image containing CARLA encoded depth-map to a 2D array containing
        the depth value of each pixel normalized between [0.0, 1.0].
        """
        array = self._to_bgra_array(image)
        array = array.astype(np.float32)
        # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
        normalized_depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
        normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)
        return normalized_depth
