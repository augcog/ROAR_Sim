from roar_autonomous_system.control.controller import Controller
from roar_autonomous_system.util.models import Control, Vehicle, Transform, Location

class VehicleMPCController(Controller):
    def __init__(self, 
                 vehicle: Vehicle, 
                 target_speed=float("inf"),
                 steps_ahead=10,
                 dt=0.1):
        super.__init__(vehicle)
    
    def run_step(self, next_waypoint: Transform) -> Control:
        pass

    def sync(self):
        pass