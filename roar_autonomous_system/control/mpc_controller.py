# References:
# https://github.com/asap-report/carla/blob/racetrack/PythonClient/racetrack/model_predictive_control.py

""" This module contains MPC controller. """

from roar_autonomous_system.control.controller import Controller
from roar_autonomous_system.util.models import Control, Vehicle, Transform, Location


class VehicleMPCController(Controller):
    def __init__(self, 
                 vehicle: Vehicle, 
                 target_speed=float("inf"),
                 steps_ahead=10,
                 dt=0.1):
        super.__init__(vehicle)
        self.target_speed = target_speed
        self.state_vars = ('x', 'y', 'v', 'ψ', 'cte', 'eψ')

        self.steps_ahead = steps_ahead
        self.dt = dt

        # Cost function coefficients
        self.cte_coeff = 100  # 100
        self.epsi_coeff = 100  # 100
        self.speed_coeff = 0.4  # 0.2
        self.acc_coeff = 1  # 1
        self.steer_coeff = 0.1  # 0.1
        self.consec_acc_coeff = 50
        self.consec_steer_coeff = 50

        # Front wheel L
        self.Lf = 2.5

        # How the polynomial fitting the desired curve is fitted
        self.steps_poly = 30
        self.poly_degree = 3

        # Bounds for the optimizer
        self.bounds = (
                6 * self.steps_ahead * [(None, None)]
                + self.steps_ahead * [THROTTLE_BOUNDS]
                + self.steps_ahead * [STEER_BOUNDS]
        )


    
    def run_step(self, next_waypoint: Transform) -> Control:
        pass

    def sync(self):
        pass
