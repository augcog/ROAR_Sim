from pydantic import BaseModel, Field
from typing import Union
import math
from roar_autonomous_system.utilities_module.data_structures_models import Transform, Vector3D
import numpy as np


class VehicleControl(BaseModel):
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


class Vehicle(BaseModel):
    """
    Encodes the Vehicle's state at the last tick
    """
    velocity: Vector3D
    transform: Union[Transform, None] = Field(default=None)
    control: VehicleControl  # ?

    @staticmethod
    def get_speed(vehicle):
        """
        Compute speed of a vehicle in Km/h.

            :param vehicle: the vehicle for which speed is calculated
            :return: speed as a float in Km/h
        """
        vel = vehicle.velocity
        return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def get_matrix(self):
        """
        Calculate extrinsics matrix with respect to parent object
        http://planning.cs.uiuc.edu/node104.html

        Returns:

        """
        location = self.transform.location
        rotation = self.transform.rotation
        yaw, pitch, roll = rotation.yaw, rotation.pitch, rotation.roll
        tx, ty, tz = location.x, location.y, location.z
        c_y = np.cos(np.radians(yaw))
        s_y = np.sin(np.radians(yaw))
        c_r = np.cos(np.radians(roll))
        s_r = np.sin(np.radians(roll))
        c_p = np.cos(np.radians(pitch))
        s_p = np.sin(np.radians(pitch))
        matrix = np.identity(4)
        matrix[0, 3] = tx
        matrix[1, 3] = ty
        matrix[2, 3] = tz
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix
