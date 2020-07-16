# References:
# https://github.com/asap-report/carla/blob/racetrack/PythonClient/racetrack/model_predictive_control.py

""" This module contains MPC controller. """

import logging
import numpy as np
import sympy as sym

from sympy.tensor.array import derive_by_array
from roar_autonomous_system.control.controller import Controller
from roar_autonomous_system.util.models import Control, Vehicle, Transform, Location


# set up pretty print
# sym.init_printing()


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
                 target_speed=float("inf"),
                 steps_ahead=10,
                 max_throttle=1,
                 max_steering=1,
                 dt=0.1):
        super().__init__(vehicle)
        self.logger = logging.getLogger(__name__)

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
        self.steer = None
        self.throttle = None

        self.logger.debug("MPC Controller initiated")
        self.logger.debug(f"  cost_func:      {self.cost_func}")
        self.logger.debug(f"  cost_grad_func: {self.cost_grad_func}")
        self.logger.debug(f"  constr_funcs:   {self.constr_funcs}")

    def run_step(self, next_waypoint: Transform) -> Control:
        # self.logger.debug("Using MPC run_step")
        return Control()

    def sync(self):
        pass

    def get_func_constraints_and_bounds(self):
        """
        Defines MPC's cost function and constraints.
        """
        # Polynomial coefficients will also be symbolic variables
        poly = self.create_array_of_symbols('poly', self.poly_degree + 1)

        # Initialize the initial state
        x_init = sym.symbols('x_init')
        y_init = sym.symbols('y_init')
        ψ_init = sym.symbols('ψ_init')
        v_init = sym.symbols('v_init')
        cte_init = sym.symbols('cte_init')
        eψ_init = sym.symbols('eψ_init')

        init = (x_init, y_init, ψ_init, v_init, cte_init, eψ_init)

        # State variables
        x = self.create_array_of_symbols('x', self.steps_ahead)
        y = self.create_array_of_symbols('y', self.steps_ahead)
        ψ = self.create_array_of_symbols('ψ', self.steps_ahead)
        v = self.create_array_of_symbols('v', self.steps_ahead)
        cte = self.create_array_of_symbols('cte', self.steps_ahead)
        eψ = self.create_array_of_symbols('eψ', self.steps_ahead)

        # Actuators
        a = self.create_array_of_symbols('a', self.steps_ahead)
        δ = self.create_array_of_symbols('δ', self.steps_ahead)

        vars_ = (
            # Symbolic arrays (but NOT actuators)
            *x, *y, *ψ, *v, *cte, *eψ,

            # Symbolic arrays (actuators)
            *a, *δ,
        )

        cost = 0
        for t in range(self.steps_ahead):
            cost += (
                # Reference state penalties
                    self.cte_coeff * cte[t] ** 2
                    + self.epsi_coeff * eψ[t] ** 2 +
                    + self.speed_coeff * (v[t] - self.target_speed) ** 2

                    # Actuator penalties
                    + self.acc_coeff * a[t] ** 2
                    + self.steer_coeff * δ[t] ** 2
            )

        # Penalty for differences in consecutive actuators
        for t in range(self.steps_ahead - 1):
            cost += (
                    self.consec_acc_coeff * (a[t + 1] - a[t]) ** 2
                    + self.consec_steer_coeff * (δ[t + 1] - δ[t]) ** 2
            )

        # Initialize constraints
        eq_constr = _EqualityConstraints(self.steps_ahead, self.state_vars)
        eq_constr['x'][0] = x[0] - x_init
        eq_constr['y'][0] = y[0] - y_init
        eq_constr['ψ'][0] = ψ[0] - ψ_init
        eq_constr['v'][0] = v[0] - v_init
        eq_constr['cte'][0] = cte[0] - cte_init
        eq_constr['eψ'][0] = eψ[0] - eψ_init

        for t in range(1, self.steps_ahead):
            curve = sum(poly[-(i+1)] * x[t-1]**i for i in range(len(poly)))
            # The desired ψ is equal to the derivative of the polynomial curve at
            #  point x[t-1]
            ψdes = sum(poly[-(i+1)] * i*x[t-1]**(i-1) for i in range(1, len(poly)))

            eq_constr['x'][t] = x[t] - (x[t-1] + v[t-1] * sym.cos(ψ[t-1]) * self.dt)
            eq_constr['y'][t] = y[t] - (y[t-1] + v[t-1] * sym.sin(ψ[t-1]) * self.dt)
            eq_constr['ψ'][t] = ψ[t] - (ψ[t-1] - v[t-1] * δ[t-1] / self.Lf * self.dt)
            eq_constr['v'][t] = v[t] - (v[t-1] + a[t-1] * self.dt)
            eq_constr['cte'][t] = cte[t] - (curve - y[t-1] + v[t-1] * sym.sin(eψ[t-1]) * self.dt)
            eq_constr['eψ'][t] = eψ[t] - (ψ[t-1] - ψdes - v[t-1] * δ[t-1] / self.Lf * self.dt)

        # Generate actual functions from
        cost_func = self.generate_fun(cost, vars_, init, poly)
        cost_grad_func = self.generate_grad(cost, vars_, init, poly)

        constr_funcs = []
        for symbol in self.state_vars:
            for t in range(self.steps_ahead):
                func = self.generate_fun(eq_constr[symbol][t], vars_, init, poly)
                grad_func = self.generate_grad(eq_constr[symbol][t], vars_, init, poly)
                constr_funcs.append(
                    {'type': 'eq', 'fun': func, 'jac': grad_func, 'args': None},
                )

        return cost_func, cost_grad_func, constr_funcs

    def generate_fun(self, symb_fun, vars_, init, poly):
        """
        Generates a function of the form `fun(x, *args)`
        """
        args = init + poly
        return sym.lambdify((vars_, *args), symb_fun, self.evaluator)

    def generate_grad(self, symb_fun, vars_, init, poly):
        """
        TODO: add comments
        """
        args = init + poly
        return sym.lambdify(
            (vars_, *args),
            derive_by_array(symb_fun, vars_ + args)[:len(vars_)],
            self.evaluator
        )

    @staticmethod
    def create_array_of_symbols(str_symbol, N):
        return sym.symbols('{symbol}0:{N}'.format(symbol=str_symbol, N=N))
