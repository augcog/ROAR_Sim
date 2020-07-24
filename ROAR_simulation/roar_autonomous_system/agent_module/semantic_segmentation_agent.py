from ROAR_simulation.roar_autonomous_system.agent_module.agent import Agent
from ROAR_simulation.roar_autonomous_system.utilities_module.vehicle_models \
    import \
    Vehicle, VehicleControl
from ROAR_simulation.roar_autonomous_system.utilities_module \
    .data_structures_models import \
    SensorsData
from ROAR_simulation.roar_autonomous_system.perception_module \
    .ground_plane_detector import \
    SemanticSegmentationDetector
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    VehiclePIDController
from ROAR_simulation.roar_autonomous_system.control_module.pid_controller \
    import \
    PIDParam
from ROAR_simulation.roar_autonomous_system.planning_module.local_planner \
    .gpd_only_local_planner import \
    SemanticSegmentationOnlyPlanner
from ROAR_simulation.roar_autonomous_system.visualization_module.visualizer \
    import \
    Visualizer
from ROAR_simulation.roar_autonomous_system.configurations.agent_settings \
    import \
    AgentSettings


class SemanticSegmentationAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentSettings):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings)
        self.semantic_seg_detector = \
            SemanticSegmentationDetector(vehicle=vehicle,
                                         camera=self.front_depth_camera)
        self.controller = \
            VehiclePIDController(self.vehicle,
                                 args_lateral=PIDParam.default_lateral_param(),
                                 args_longitudinal=PIDParam.
                                 default_longitudinal_param(),
                                 target_speed=40)
        self.local_planner = \
            SemanticSegmentationOnlyPlanner(vehicle=self.vehicle,
                                            controller=self.controller,
                                            gpd_detector=
                                            self.semantic_seg_detector,
                                            next_waypoint_distance=10,
                                            max_turn_degree=10)
        self.visualizer = Visualizer(agent=self)

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(SemanticSegmentationAgent, self).run_step(vehicle=vehicle,
                                                        sensors_data=
                                                        sensors_data)
        self.semantic_seg_detector.run_step(vehicle=vehicle,
                                            new_data=sensors_data.
                                            front_depth.data)
        control = self.local_planner.run_step(vehicle=vehicle)
        self.visualizer.visualize_semantic_segmentation(
            semantic_segmetation=
            self.semantic_seg_detector.semantic_segmentation)
        return control
