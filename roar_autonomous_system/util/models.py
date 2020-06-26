from pydantic import BaseModel, Field
from scipy.spatial import distance
import numpy as np
from typing import Union
import math

class Location(BaseModel):
    x: float = Field(..., title="X coordinate", description="Distance in meters from origin to spot on X axis")
    y: float = Field(..., title="Y coordinate", description="Distance in meters from origin to spot on Y axis")
    z: float = Field(..., title="Z coordinate", description="Distance in meters from origin to spot on Z axis")

    def distance(self, other_location):
        """Euclidean distance"""
        return distance.euclidean((self.x, self.y, self.z), (other_location.x, other_location.y, other_location.z))

    def __add__(self, other):
        return Location(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __str__(self):
        return f"{self.x:.3},{self.y:.3},{self.z:.3}"


class Rotation(BaseModel):
    pitch: float = Field(..., title="Pitch", description="Degree around the Y-axis")
    yaw: float = Field(..., title="Yaw", description="Degree around the Z-axis")
    roll: float = Field(..., title="Roll", description="Degree around the X-axis")

    def __str__(self):
        return f"{self.pitch},{self.yaw},{self.roll}"

class Transform(BaseModel):
    location: Location = Field(default=Location(x=0, y=0, z=0))
    rotation: Rotation = Field(default=Rotation(pitch=0, yaw=0, roll=0))


class Control(BaseModel):
    throttle: float = Field(default=0)
    steering: float = Field(default=0)

    @staticmethod
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)

    def get_throttle(self) -> float:
        """
        Cap it between -1  and 1
        :return:
        """
        return self.clamp(self.throttle, -1, 1)

    def get_steering(self) -> float:
        return self.clamp(self.steering, -1, 1)


class RGBData(BaseModel):
    data: np.ndarray = Field(..., title="RGB Data", description="Array of BGRA 32-bit pixels")

    class Config:
        arbitrary_types_allowed = True


class DepthData(BaseModel):
    data: np.ndarray = Field(..., title="Depth Data", description="Array of BRGA 32 bit pixels???")

    class Config:
        arbitrary_types_allowed = True


class Vector3D(BaseModel):
    x: float = Field(default=0)
    y: float = Field(default=0)
    z: float = Field(default=0)


class IMUData(BaseModel):
    accelerometer: Vector3D = Field(default=Vector3D(), title="Accelerometer data",
                                    description="Linear acceleration in m/s^2")
    gyroscope: Vector3D = Field(default=Vector3D(), title="Gyroscope data", description="Angular velocity in rad/sec")


class SensorData(BaseModel):
    rgb: RGBData = Field(default=np.asarray([]))
    depth: DepthData = Field(default=np.asarray([]))
    imu_data = Field(default=IMUData())


class Vehicle(BaseModel):
    """
    Encodes the Vehicle's state at the last tick
    """
    velocity: Vector3D
    transform: Union[Transform, None] = Field(default=None)
    control: Control  # ?

    @staticmethod
    def get_speed(vehicle):
        """
        Compute speed of a vehicle in Km/h.

            :param vehicle: the vehicle for which speed is calculated
            :return: speed as a float in Km/h
        """
        vel = vehicle.velocity
        return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


