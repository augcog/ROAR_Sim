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


