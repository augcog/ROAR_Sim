from abc import ABC, abstractmethod
import logging
from roar_autonomous_system.planning_module.abstract_planner import AbstractPlanner
from roar_autonomous_system.utilities_module.vehicle_models import Vehicle


class BehaviorPlanner(AbstractPlanner):
    def run_step(self, vehicle: Vehicle):
        pass