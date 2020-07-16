from pydantic import BaseModel, Field


class PIDParam(BaseModel):
    K_P: float = Field(default=1)
    K_D: float = Field(default=1)
    K_I: float = Field(default=1)
    dt: float = Field(default=1)

    @staticmethod
    def default_lateral_param():
        return PIDParam(K_P=1.95, K_D=0.2, K_I=0.07, dt=1.0/20.0)

    @staticmethod
    def default_longitudinal_param():
        return PIDParam(K_P=1, K_D=0, K_I=0.05, dt=1.0/20.0)


# speed - LateralPIDParam
OPTIMIZED_LATERAL_PID_VALUES = {
    60: PIDParam(K_P=0.3, K_D=0.3, K_I=0.2),
    100: PIDParam(K_P=0.2, K_D=0.2, K_I=0.5),
    150: PIDParam(K_P=0.01, K_D=0.075, K_I=0.7),
}

# MPC-RELATED
# Constraints for MPC
STEER_BOUND = 1.0
STEER_BOUNDS = (-STEER_BOUND, STEER_BOUND)
THROTTLE_BOUND = 1.0
THROTTLE_BOUNDS = (0, THROTTLE_BOUND)
