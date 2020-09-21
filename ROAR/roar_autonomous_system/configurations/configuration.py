from pydantic import BaseModel, Field
from ROAR.roar_autonomous_system.configurations.agent_settings import AgentConfig
from ROAR.carla_client.carla_settings import CarlaConfig
from ROAR.ROAR_Jetson.jetson_config import JetsonConfig
from typing import Optional


class Configuration(BaseModel):
    agent_config: AgentConfig = Field(default=AgentConfig())
    carla_config: CarlaConfig = Field(default=CarlaConfig())
    jetson_config: Optional[JetsonConfig] = Field(default=None)
