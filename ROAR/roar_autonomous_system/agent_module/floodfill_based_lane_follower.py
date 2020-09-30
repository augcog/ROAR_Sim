from ROAR.roar_autonomous_system.agent_module.agent import Agent, AgentConfig
from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.roar_autonomous_system.control_module.pid_controller import VehiclePIDController
from ROAR.roar_autonomous_system.perception_module.flood_fill_lane_detector import FloodfillLaneDetector
from ROAR.roar_autonomous_system.control_module.pid_controller import PIDParam
import cv2
import numpy as np


class FloodfillBasedLaneFollower(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle, agent_settings)
        self.controller = VehiclePIDController(agent=self, args_lateral=PIDParam.default_lateral_param(),
                                               args_longitudinal=PIDParam.default_longitudinal_param())
        self.floodfill_lane_detector = FloodfillLaneDetector(agent=self)

    def run_step(self, sensors_data: SensorsData, vehicle: Vehicle) -> VehicleControl:
        super().run_step(sensors_data=sensors_data, vehicle=vehicle)
        try:
            img = self.floodfill_lane_detector.run_step()

            # left, front, right dot img location
            left_dot_coord = (self.front_rgb_camera.image_size_x // 4, 400)
            center_dot_coord = (self.front_rgb_camera.image_size_x // 2, 400)
            right_dot_coord = (self.front_rgb_camera.image_size_x - (self.front_rgb_camera.image_size_x // 4), 400)
            blue = [255,0,0]

            left_ok = self._is_equal(img[left_dot_coord[::-1]], blue)
            center_ok = self._is_equal(img[center_dot_coord[::-1]], blue)
            right_ok = self._is_equal(img[right_dot_coord[::-1]], blue)

            if center_ok:
                return VehicleControl(throttle=0.5, steering=0)
            elif left_ok:
                return VehicleControl(throttle=0.3, steering=-0.5)
            elif right_ok:
                return VehicleControl(throttle=0.3, steering=0.5)
            else:
                return VehicleControl()
        except:
            return VehicleControl()
    @staticmethod
    def _is_equal(arr1, arr2):
        # print(sum(arr1 == arr2))
        return np.alltrue(arr1 == arr2)