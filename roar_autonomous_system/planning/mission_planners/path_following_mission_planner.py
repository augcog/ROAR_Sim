from roar_autonomous_system.planning.mission_planners.mission_planner import MissionPlanner
from pathlib import Path
import logging
from typing import List, Optional
from roar_autonomous_system.util.models import Transform, Location, Rotation


class PathFollowingMissionPlanner(MissionPlanner):
    """
    A mission planner that takes in a file that contains x,y,z coordinates, formulate into carla.Transform
    """

    def __init__(self, file_path: Path):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.file_path = file_path
        self.mission_plan = self.update_mission_plan()
        self.logger.debug("Path Following Mission Planner Initiated")

    def update_mission_plan(self) -> List[Transform]:
        """
        Generates a list of waypoints based on the input file path

        :return a list of waypoint
        """
        raw_path: List[List[float]] = self._read_data_file()
        for coord in raw_path:
            if len(coord) == 3 or len(coord) == 6:
                self.mission_plan.append(self._raw_coord_to_transform(coord))
        # result = [self._raw_coord_to_transform(coord) for coord in raw_path if len(coord) == 3 or len(coord) == 6]
        self.logger.debug(f"Computed Mission path of length {len(self.mission_plan)}")
        return self.mission_plan

    def _read_data_file(self) -> List[List[float]]:
        """
        Read data file and generate a list of (x, y, z) where each of x, y, z is of type float
        """
        result = []
        with open(self.file_path.as_posix(), 'r') as f:
            for line in f:
                result.append(self._read_line(line=line))
        return result

    def _raw_coord_to_transform(self, raw: List[float]) -> Optional[Transform]:
        if len(raw) == 3:
            return Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
        elif len(raw) == 6:
            return Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]),
                             rotation=Rotation(pitch=raw[4], yaw=raw[5], roll=raw[6]))
        else:
            self.logger.error(f"Point {raw} is invalid, skipping")
            return None

    def _read_line(self, line: str) -> List[float]:
        """
        parse a line of string of "x,y,z" into [x,y,z]

        """
        x, y, z = line.split(",")
        x, y, z = float(x), float(y), float(z)
        return [x, y, z]
