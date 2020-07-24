# References:
# https://github.com/asap-report/carla/blob/racetrack/PythonClient/racetrack/model_predictive_control.py

""" This module contains MPC controller. """

import logging
import numpy as np
import pandas as pd
import random
import sympy as sym

from pathlib import Path
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
from sympy.tensor.array import derive_by_array

from ROAR_simulation.roar_autonomous_system.control_module.controller import Controller
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import VehicleControl, Vehicle
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import Transform, Location

class _EqualityConstraints(object):
    """Class for storing equality constraints in the MPC."""

    def __init__(self, N, state_vars):
        self.dict = {}
        for symbol in state_vars:
            self.dict[symbol] = N*[None]

    def __getitem__(self, key):
        return self.dict[key]

    def __setitem__(self, key, value):
        self.dict[key] = value

class VehicleMPCController(Controller):
    def __init__(self,
                 vehicle: Vehicle,
                 route_file_path: Path, # read in route
                 target_speed=float("inf"),
                 steps_ahead=10,
                 max_throttle=1,
                 max_steering=1,
                 dt=0.1):
        super().__init__(vehicle)
        self.logger = logging.getLogger(__name__)

        # Read in route file
        self.track_DF = pd.read_csv(route_file_path, header=None)
        # Fit the route to a curve
        spline_points = 10000
        self.pts_2D = self.track_DF.loc[:, [0, 1]].values
        tck, u = splprep(self.pts_2D.T, u=None, s=2.0, per=1, k=3)
        u_new = np.linspace(u.min(), u.max(), spline_points)
        x_new, y_new = splev(u_new, tck, der=0)
        self.pts_2D = np.c_[x_new, y_new]

        # Modified parm
        self.prev_cte = 0

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
        self.steps_poly = 30 # modify to 3 when using 3D data
        self.poly_degree = 3

        # Bounds for the optimizer
        self.bounds = (
                6 * self.steps_ahead * [(None, None)]
                + self.steps_ahead * [(0, max_throttle)] # throttle bounds
                + self.steps_ahead * [(-max_steering, max_steering)] # steer bounds
        )

        # State 0 placeholder
        num_vars = (len(self.state_vars) + 2)  # State variables and two actuators
        self.state0 = np.zeros(self.steps_ahead * num_vars)

        # Lambdify and minimize stuff
        self.evaluator = 'numpy'
        self.tolerance = 1
        self.cost_func, self.cost_grad_func, self.constr_funcs = self.get_func_constraints_and_bounds()

        # To keep the previous state
        self.steer = 0
        self.throttle = 0

        self.logger.debug("MPC Controller initiated")
        # self.logger.debug(f"  cost_func:      {self.cost_func}")
        # self.logger.debug(f"  cost_grad_func: {self.cost_grad_func}")
        # self.logger.debug(f"  constr_funcs:   {self.constr_funcs}")
    
    