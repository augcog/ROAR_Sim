from ROAR_simulation.roar_autonomous_system.planning_module.local_planner.local_planner import (
    LocalPlanner,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    Vehicle,
)
from ROAR_simulation.roar_autonomous_system.control_module.controller import Controller
from ROAR_simulation.roar_autonomous_system.perception_module.semantic_segmentation_detector import (
    SemanticSegmentationDetector,
)
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import (
    VehicleControl,
)
import numpy as np


class SemanticSegmentationOnlyPlanner(LocalPlanner):
    def __init__(
        self,
        vehicle: Vehicle,
        controller: Controller,
        gpd_detector: SemanticSegmentationDetector,
        next_waypoint_distance: float = 10,
        max_turn_degree: int = 10,
    ):
        super().__init__(vehicle, controller)
        self.gpd_detector = gpd_detector
        self._next_waypoint_distance = next_waypoint_distance
        self._max_turn_degree = max_turn_degree
        self._carla_distance_to_pixel_scaling = 10

    def run_step(self, vehicle: Vehicle) -> VehicleControl:
        """
        Get data from GPD_Detector
            if front is clear
                put a waypoint in front of the vehicle
            if front is blocked, both left and right are clear,
                put a point at random to the left or right of the vehicle
            if front is blocked, left is blocked, right is clear
                put a point to the right of the vehicle
            if front is blocked, right is blocked, left is clear
                put a point to the left of the vehicle
            if front, right, and left are all blocked
                stop (return a control that does nothing)
        Returns:
            VehicleControl object
        """
        super(SemanticSegmentationOnlyPlanner, self).run_step(vehicle=vehicle)
        if self.gpd_detector.curr_ground is None:
            return VehicleControl()
        curr_location = self.vehicle.transform.location
        is_front_clear = self.is_front_clear()
        is_left_clear = self.is_left_clear()
        is_right_clear = self.is_right_clear()
        copy_depth = self.gpd_detector.semantic_segmentation.copy()
        copy_depth[320:600, 200 - 10 : 200 + 10] = 255  # left
        copy_depth[330:600, 400 - 10 : 400 + 10] = 255  # straight
        copy_depth[320:600, 600 - 10 : 600 + 10] = 255  # right
        next_way_point = None
        if is_front_clear:
            # generate front waypoint
            # print("Going Straight")
            return VehicleControl(throttle=0.75, steering=0)

        elif is_right_clear:
            # print("Turning Right")
            return VehicleControl(throttle=0.5, steering=0.2)
        elif is_left_clear:
            # print("Turning Left")
            return VehicleControl(throttle=0.5, steering=-0.2)
        else:
            # print("I am stucked")
            return VehicleControl()

    def is_done(self):
        return False

    def set_mission_plan(self):
        pass

    def sync(self):
        pass

    def is_front_clear(self) -> bool:
        """
        Take a block of 10 x 10 in next_waypoint_distance front and see if they are all NOT white.
        Returns:
            True if the block is all None white, false otherwise
        """
        # center_pixel = self._next_waypoint_distance * self._carla_distance_to_pixel_scaling  # this variable is guessing
        # Y, X, _ = np.shape(self.gpd_detector.curr_depth.data)
        X, Y = 400, 324
        factor = 10
        ground_section = self.gpd_detector.curr_ground[320:600, X - factor : X + factor]
        return np.all(ground_section)

    def is_left_clear(self) -> bool:
        """
        Take a block of 10 x 10 to degree left, and next_waypoint_distance and see if they are all NOT white.
        Returns:
            True if the block is all None white, false otherwise
        """
        X, Y = 200, 324
        factor = 10
        ground_section = self.gpd_detector.curr_ground[330:600, X - factor : X + factor]
        return np.all(ground_section)

    def is_right_clear(self) -> bool:
        """
        Take a block of 10 x 10 to degree right, and next_waypoint_distance and see if they are all NOT white.
        Returns:
            True if the block is all None white, false otherwise
        """
        X, Y = 600, 324
        factor = 10

        ground_section = self.gpd_detector.curr_ground[320:600, X - factor : X + factor]
        return np.all(ground_section)
