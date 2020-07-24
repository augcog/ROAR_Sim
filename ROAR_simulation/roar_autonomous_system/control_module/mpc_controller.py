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

