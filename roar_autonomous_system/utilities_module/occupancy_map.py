import numpy as np
from pydantic import BaseModel, Field
from typing import Union, List, Tuple
from roar_autonomous_system.utilities_module.data_structures_models import Transform, Location
import cv2


class OccupancyMap(BaseModel):
    data: np.ndarray = Field(..., title="Occu Map Data", description="2D array containing probability of whether there is an obstacle. ")
    road_width: int = Field(default=15)
    map_additional_padding: int = Field(20)
    x_min: int = Field(default=0)
    y_min: int = Field(default=0)

    class Config:
        arbitrary_types_allowed = True

    @staticmethod
    def create_map(mission_plan: Union[List[Transform], None] = None,
                   map_width: int = 1000,
                   map_height: int = 1000,
                   road_width: int = 15,
                   map_additional_padding: int = 40):
        if mission_plan is None:
            occupancy_grid = np.ones((map_width + map_additional_padding, map_height + map_additional_padding))
            return OccupancyMap(data=occupancy_grid, map_additional_padding=map_additional_padding,
                                road_width=road_width)
        else:
            # convert mission plan back to np array
            raw_data = np.asarray([[m.location.x, m.location.y] for m in mission_plan])
            X = np.ravel(raw_data[:, :1])
            Y = np.ravel(raw_data[:, 1:2])
            x_min = min(X)
            y_min = min(Y)
            X_transformed = np.ceil((X - x_min) + map_additional_padding).astype(int)
            Y_transformed = np.ceil((Y - y_min) + map_additional_padding).astype(int)
            occupancy_grid = np.ones(
                (max(X_transformed) + map_additional_padding, max(Y_transformed) + map_additional_padding))
            for x, y in zip(X_transformed, Y_transformed):
                occupancy_grid[x - road_width // 2: x + road_width // 2, y - road_width // 2: y + road_width // 2] = 0
            return OccupancyMap(data=occupancy_grid, road_width=road_width,
                                map_additional_padding=map_additional_padding, x_min=x_min, y_min=y_min)

    def convert_to_map_coord(self, location: Location) -> Tuple[int, int, int]:
        return np.ceil(location.x - self.x_min + self.map_additional_padding).astype(int), \
               np.ceil(location.y - self.y_min + self.map_additional_padding).astype(int), \
               int(location.z)

    def convert_from_map_coord(self, coord: Tuple[int, int]) -> Location:
        x = coord[0] - self.map_additional_padding + self.x_min
        y = coord[1] - self.map_additional_padding + self.y_min
        return Location(x=x, y=y, z=0)

    def visualize(self, center_coord: Union[None, Tuple[int, int]] = None, fov: int = 20, duration=1):
        """
        draw the map
        :param center_coord: focus on center_coord, if None, draw out the entire map centered at what ever center it should be
        :param fov: only map out this amount of area
        :param duration: display it for duration miliseconds
        :return:
            None
        """
        if center_coord is None:
            cv2.imshow("Occupancy Grid Map", self.data)
        else:
            vehicle_width = 5
            x = center_coord[0]
            y = center_coord[1]
            vis_data = self.data.copy()
            coord_values = vis_data[x - vehicle_width: x+vehicle_width, y - vehicle_width: y + vehicle_width]
            vis_data[x - vehicle_width: x+vehicle_width, y - vehicle_width: y + vehicle_width] = 0.5
            x_min = 0 if x - fov < 0 else x-fov
            x_max = len(self.data) - 1 if x + fov > len(self.data) - 1 else x + fov
            y_min = 0 if y - fov < 0 else y - fov
            y_max = len(self.data[0]) - 1 if y + fov > len(self.data[0]) - 1 else y + fov
            cv2.imshow("Occupancy Grid Map", vis_data[x_min:x_max, y_min:y_max])
            vis_data[x - vehicle_width: x+vehicle_width, y - vehicle_width: y + vehicle_width] = coord_values

        cv2.waitKey(duration)
