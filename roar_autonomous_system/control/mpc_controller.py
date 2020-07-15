from roar_autonomous_system.control.controller import Controller
from roar_autonomous_system.util.models import Control, Vehicle, Transform, Location

class VehicleMPCController(Controller):
    def __init__(self, vehicle: Vehicle)