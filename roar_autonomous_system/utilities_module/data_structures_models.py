from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial import distance
from typing import Union


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

    def to_array(self) -> np.array :
        return np.array([self.x, self.y, self.z])


class Rotation(BaseModel):
    pitch: float = Field(..., title="Pitch", description="Degree around the Y-axis")
    yaw: float = Field(..., title="Yaw", description="Degree around the Z-axis")
    roll: float = Field(..., title="Roll", description="Degree around the X-axis")

    def __str__(self):
        return f"{self.pitch},{self.yaw},{self.roll}"

    def to_array(self) -> np.array:
        return np.array([self.pitch, self.yaw, self.roll])

class Transform(BaseModel):
    location: Location = Field(default=Location(x=0, y=0, z=0))
    rotation: Rotation = Field(default=Rotation(pitch=0, yaw=0, roll=0))


class Vector3D(BaseModel):
    x: float = Field(default=0)
    y: float = Field(default=0)
    z: float = Field(default=0)


class RGBData(BaseModel):
    data: np.ndarray = Field(..., title="RGB Data", description="Array of size (WIDTH, HEIGHT, 3)")

    class Config:
        arbitrary_types_allowed = True


class DepthData(BaseModel):
    data: np.ndarray = Field(..., title="Depth Data", description="Array of size (WIDTH, HEIGHT, 3)")

    class Config:
        arbitrary_types_allowed = True


class IMUData(BaseModel):
    accelerometer: Vector3D = Field(default=Vector3D(), title="Accelerometer data",
                                    description="Linear acceleration in m/s^2")
    gyroscope: Vector3D = Field(default=Vector3D(), title="Gyroscope data", description="Angular velocity in rad/sec")


class SensorsData(BaseModel):
    front_rgb: Union[RGBData, None] = Field(default=None)
    rear_rgb: Union[RGBData, None] = Field(default=None)
    front_depth: Union[DepthData, None] = Field(default=None)
    imu_data: Union[IMUData, None] = Field(default=None)
