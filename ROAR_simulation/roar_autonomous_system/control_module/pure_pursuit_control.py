from ROAR_simulation.roar_autonomous_system.control_module.controller import Controller
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform
import numpy as np
import math


class PurePursuitController(Controller):
    """
    Citation: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
    """

    def __init__(self, vehicle: Vehicle, look_ahead_gain: float = 0.1,
                 look_ahead_distance: float = 2, target_speed=60):
        """
        Pure Pursuit controller.
        Args:
            vehicle: current vehicle state
            look_ahead_gain: multiply with current velocity to achieve the effect of driving faster = looking further.
            Normally ranges from 0 - 1
            look_ahead_distance: base look forward distance
        """
        super(PurePursuitController, self).__init__(vehicle=vehicle)
        self.target_speed = target_speed
        self.look_ahead_gain = look_ahead_gain
        self.look_ahead_distance = look_ahead_distance
        self.latitunal_controller = LatitunalPurePursuitController(vehicle=self.vehicle,
                                                                   look_ahead_gain=look_ahead_gain,
                                                                   look_ahead_distance=look_ahead_distance)
        self.longitunal_controller = LongitunalPurePursuitController(vehicle=self.vehicle, target_speed=target_speed)

    def run_step(self, vehicle: Vehicle, next_waypoint: Transform, **kwargs) -> VehicleControl:
        control = VehicleControl(throttle=self.longitunal_controller.run_step(vehicle=vehicle),
                                 steering=self.latitunal_controller.run_step(vehicle=vehicle,
                                                                             next_waypoint=next_waypoint))
        return control


class LongitunalPurePursuitController:
    def __init__(self, vehicle: Vehicle, target_speed=60, kp=0.1):
        self.vehicle = vehicle
        self.target_speed = target_speed
        self.kp = kp

    def run_step(self, vehicle: Vehicle) -> float:
        self.sync(vehicle=vehicle)
        return float(VehicleControl.clamp(self.kp * (self.target_speed - Vehicle.get_speed(vehicle)), 0, 1))

    def sync(self, vehicle: Vehicle):
        self.vehicle = vehicle


class LatitunalPurePursuitController:
    def __init__(self, vehicle: Vehicle, look_ahead_gain: float, look_ahead_distance: float):
        self.vehicle = vehicle
        self.look_ahead_gain = look_ahead_gain
        self.look_ahead_distance = look_ahead_distance

    def run_step(self, vehicle: Vehicle, next_waypoint: Transform) -> float:
        self.sync(vehicle=vehicle)
        target_y = next_waypoint.location.y
        target_x = next_waypoint.location.x
        angle_difference = math.atan2(target_y - self.vehicle.transform.location.y,
                                      target_x - self.vehicle.transform.location.x) - np.radians(
            self.vehicle.transform.rotation.yaw)
        curr_look_forward = self.look_ahead_gain * Vehicle.get_speed(vehicle=vehicle) + self.look_ahead_distance
        lateral_difference = math.atan2(2.0 * self.vehicle.wheel_base * math.sin(angle_difference) / curr_look_forward,
                                        1.0)
        return VehicleControl.clamp(lateral_difference, -1, 1)

    def sync(self, vehicle: Vehicle):
        self.vehicle = vehicle
