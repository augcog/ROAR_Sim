from roar_autonomous_system.agent_module.agent import Agent
from roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from roar_autonomous_system.perception_module.ground_plane_detector import GroundPlaneDetector
from roar_autonomous_system.utilities_module.camera_models import Camera
from roar_autonomous_system.control_module.pid_controller import VehiclePIDController
from roar_autonomous_system.control_module.pid_controller import PIDParam
from roar_autonomous_system.planning_module.local_planner.gpd_only_local_planner import GPDOnlyLocalPlanner


class GPDAgent(Agent):
    def __init__(self, vehicle: Vehicle, front_depth_camera: Camera, show_gpd_data=False):
        super().__init__(vehicle=vehicle, front_depth_camera=front_depth_camera)
        self.gpd_detector = GroundPlaneDetector(vehicle=vehicle, camera=self.front_depth_camera, show=show_gpd_data)
        self.controller = VehiclePIDController(self.vehicle,
                                               args_lateral=PIDParam.default_lateral_param(),
                                               args_longitudinal=PIDParam.default_longitudinal_param(),
                                               target_speed=40)
        self.local_planner = GPDOnlyLocalPlanner(vehicle=self.vehicle,
                                                 controller=self.controller,
                                                 gpd_detector=self.gpd_detector,
                                                 next_waypoint_distance=10,
                                                 max_turn_degree=10)

    def run_step(self, vehicle: Vehicle, sensors_data: SensorsData) -> VehicleControl:
        super(GPDAgent, self).run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.gpd_detector.run_step(vehicle=vehicle, new_data=sensors_data.front_depth.data)
        control = self.local_planner.run_step(vehicle=vehicle)
        return control
