from roar_autonomous_system.planning.local_planners.local_planner import LocalPlanner
from roar_autonomous_system.util.models import Vehicle, Transform, Control
from roar_autonomous_system.control.controller import Controller
from roar_autonomous_system.planning.mission_planners.mission_planner import MissionPlanner
from roar_autonomous_system.planning.behavior_planners.behavior_planner import BehaviorPlanner
import logging
from typing import Union
from roar_autonomous_system.util.errors import AgentException


class SimplePathFollowingLocalPlanner(LocalPlanner):
    def __init__(self, vehicle: Vehicle, controller: Controller, mission_planner: MissionPlanner,
                 behavior_planner: BehaviorPlanner, closeness_threshold=0.5):
        super().__init__(vehicle, controller, mission_planner, behavior_planner, closeness_threshold)
        self.logger = logging.getLogger("SimplePathFollowingLocalPlanner")
        self.set_mission_plan()
        self.logger.debug("Simple Path Following Local Planner Initiated")

    def set_mission_plan(self):
        """
        clears current waypoints, and reset mission plan from start

        I am simply transfering the mission plan into my waypoint queue.

        Assuming that this current run will run all the way to the end

        :return: None
        """
        self.way_points_queue.clear()
        while self.mission_planner.mission_plan:  # this actually clears the mission plan!!
            self.way_points_queue.append(self.mission_planner.mission_plan.popleft())

    def is_done(self):
        return len(self.way_points_queue) == 0

    def run_step(self) -> Control:
        self.sync()  # on every run step, sync vehicle with lower level
        if len(self.mission_planner.mission_plan) == 0 and len(self.way_points_queue) == 0:
            return Control()

        # get vehicle's location
        vehicle_transform: Union[Transform, None] = self.vehicle.transform

        if vehicle_transform is None:
            raise AgentException("I do not know where I am, I cannot proceed forward")

        # redefine closeness level based on speed
        curr_speed = Vehicle.get_speed(self.vehicle)
        if curr_speed < 60:
            self.closeness_threshold = 5
        elif curr_speed < 80:
            self.closeness_threshold = 15
        elif curr_speed < 120:
            self.closeness_threshold = 20
        else:
            self.closeness_threshold = 50

        # get current waypoint
        curr_closest_dist = float("inf")
        while True:
            if len(self.way_points_queue) == 0:
                self.logger.info("Destination reached")
                return Control()
            waypoint: Transform = self.way_points_queue[0]
            curr_dist = vehicle_transform.location.distance(waypoint.location)
            if curr_dist < curr_closest_dist:
                # if i find a waypoint that is closer to me than before
                # note that i will always enter here to start the calculation for curr_closest_dist
                curr_closest_dist = curr_dist
            elif curr_dist < self.closeness_threshold:
                # i have moved onto a waypoint, remove that waypoint from the queue
                self.way_points_queue.popleft()
            else:
                break

        target_waypoint = self.way_points_queue[0]
        # target_waypoint = Transform.average(self.way_points_queue[0], self.way_points_queue[1])
        # target_waypoint = Transform.average(self.way_points_queue[2], target_waypoint)

        control: Control = self.controller.run_step(next_waypoint=target_waypoint)
        # self.logger.debug(
        #     f"Target_Location {target_waypoint.location} "
        #     f"| Curr_Location {vehicle_transform.location} "
        #     f"| Distance {int(curr_closest_dist)}")

        return control

    def sync(self):
        self.controller.vehicle = self.vehicle
