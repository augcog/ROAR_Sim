from roar_autonomous_system.agents.agent import Agent
from roar_autonomous_system.util.models import Vehicle, SensorData, Control
from roar_autonomous_system.perception.ground_plane_detector import GroundPlaneDetector
from bridges.bridge import Bridge
from roar_autonomous_system.control.pid_controller import VehiclePIDController
from roar_autonomous_system.control.util import PIDParam
from roar_autonomous_system.planning.local_planners.gpd_only_local_planner import GPDOnlyLocalPlanner

class GPDAgent(Agent):
    def __init__(self, vehicle: Vehicle, bridge: Bridge, show_gpd_data=False):
        super().__init__(vehicle, bridge)
        self.gpd_detector = GroundPlaneDetector(show=show_gpd_data)
        self.controller = VehiclePIDController(self.vehicle,
                                               args_lateral=PIDParam.default_lateral_param(),
                                               args_longitudinal=PIDParam.default_longitudinal_param(),
                                               target_speed=40)
        self.local_planner = GPDOnlyLocalPlanner(vehicle=self.vehicle,
                                                 controller=self.controller,
                                                 gpd_detector=self.gpd_detector,
                                                 next_waypoint_distance=10,
                                                 max_turn_degree=10)

    def run_step(self, vehicle: Vehicle, sensor_data: SensorData) -> Control:
        self.sync(vehicle, sensor_data)
        self.gpd_detector.run_step()
        control = self.local_planner.run_step()
        return control

    def sync(self, vehicle: Vehicle, sensor_data: SensorData):
        self.gpd_detector.curr_depth = sensor_data.front_depth
        self.local_planner.vehicle = vehicle
