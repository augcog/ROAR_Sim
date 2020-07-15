from pydantic import BaseModel, Field
import numpy as np
from roar_autonomous_system.utilities_module.data_structures_models import Vector3D, Location, Rotation, Transform
from typing import Union, Optional, Union
import cv2


class Camera(BaseModel):
    fov: int = Field(default=70, title="Field of View")
    transform: Transform = Field(default=Transform(location=Location(x=1.6, y=0, z=1.7),
                                                   rotation=Rotation(pitch=0, yaw=0, roll=0)))
    image_size_x: int = Field(default=800, title="Image size width")
    image_size_y: int = Field(default=600, title="Image size width")
    data: Optional[np.ndarray] = Field(default=None)
    intrinsics_matrix: Optional[np.ndarray] = Field(default=None)

    def calculate_intrinsic_matrix(self):
        """
        Calculate intrinsics matrix
        Will set the attribut intrinsic matrix so that re-calculation is not necessary.

        Returns:
            Intrinsics_matrix
        """
        intrinsics_matrix = np.identity(3)
        intrinsics_matrix[0, 2] = self.image_size_x / 2.0
        intrinsics_matrix[1, 2] = self.image_size_y / 2.0
        intrinsics_matrix[0, 0] = intrinsics_matrix[1, 1] = self.image_size_y / (2.0 * np.tan(self.fov * np.pi / 360.0))
        self.intrinsics_matrix = intrinsics_matrix
        return intrinsics_matrix

    class Config:
        arbitrary_types_allowed = True

    def visualize(self, title="CameraData", duration=1):
        """
        Visualize camera data.
        Args:
            title: title of cv2 image
            duration: in milisecond

        Returns:
            None
        """
        if self.data is not None:
            cv2.imshow(title, self.data.data)
            cv2.waitKey(duration)

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