from pydantic import BaseModel, Field
import numpy as np
from roar_autonomous_system.utilities_module.data_structures_models import Vector3D, Location, Rotation
from typing import Union, Optional, Union
import cv2


class Camera(BaseModel):
    fov: int = Field(default=70, title="Field of View")
    location: Location = Field(default=Location(x=1.6, y=0, z=1.7), title="Location of sensor with respect to parent actor")
    rotation: Rotation = Field(default=Rotation(pitch=0, yaw=0, roll=0), title="Rotation of sensor with respecct to parent actor")
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
