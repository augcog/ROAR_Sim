import logging
from abc import ABC, abstractmethod
from roar_autonomous_system.planning.mission_planners.mission_planner import MissionPlanner
from roar_autonomous_system.planning.behavior_planners.behavior_planner import BehaviorPlanner
from collections import deque
from roar_autonomous_system.util.models import Vehicle
from roar_autonomous_system.control.controller import Controller


class LocalPlanner(ABC):
    def __init__(self,
                 vehicle: Vehicle,
                 controller: Controller,
                 mission_planner: MissionPlanner = None,
                 behavior_planner: BehaviorPlanner = None,
                 closeness_threshold=0.5
                 ):
        self.logger = logging.getLogger(__name__)
        self.vehicle: Vehicle = vehicle
        self.controller: Controller = controller
        self.mission_planner: MissionPlanner = mission_planner
        self.behavior_planner: BehaviorPlanner = behavior_planner
        # consider adding maxlen for performace, notice that Once a bounded length deque is full,
        # when new items are added, a corresponding number of items are discarded from the opposite end
        # https://docs.python.org/3/library/collections.html#collections.deque
        self.way_points_queue: deque = deque()
        self.closeness_threshold = closeness_threshold

    @abstractmethod
    def run_step(self):
        pass

    @abstractmethod
    def is_done(self):
        pass

    @abstractmethod
    def set_mission_plan(self):
        pass
