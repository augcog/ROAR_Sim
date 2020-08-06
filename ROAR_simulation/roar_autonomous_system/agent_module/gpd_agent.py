from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings import AgentConfig
from ROAR_simulation.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR_simulation.roar_autonomous_system.perception_module.gpd_detector import GroundPlaneDetector
from ROAR_simulation.roar_autonomous_system.utilities_module.occupancy_map import OccupancyGridMap
from ROAR_simulation.roar_autonomous_system.utilities_module.utilities import img_to_world2
import numpy as np


class GPDAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings)
        self.gpd_detector = GroundPlaneDetector(
            agent=self
        )
        self.occupancy_grid = OccupancyGridMap(absolute_maximum_map_size=800)

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(GPDAgent, self).run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.gpd_detector.run_step()

        if self.gpd_detector.curr_segmentation is not None:
            world_cords = img_to_world2(depth_img=self.front_depth_camera.data,
                                        segmentation=self.gpd_detector.curr_segmentation,
                                        criteria=self.gpd_detector.GROUND,
                                        intrinsics_matrix=self.front_depth_camera.intrinsics_matrix,
                                        extrinsics_matrix=
                                        self.front_depth_camera.transform.get_matrix() @ self.vehicle.transform.get_matrix())[:2]
            print(np.shape(world_cords))
        return VehicleControl()
