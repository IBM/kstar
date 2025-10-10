from typing import List
from pydantic import BaseModel, Field

from data.descriptions.tool_descriptions import (
    ACTIONS_DESCRIPTION,
    COST_DESCRIPTION,
    ERROR_MESSAGE_DESCRIPTION,
    LOG_DESCRIPTION,
    PLANS_DESCRIPTION,
    TIMEOUT_TRIGGERED_DESCRIPTION,
)


class Plan(BaseModel):
    cost: int = Field(default=-1, description=COST_DESCRIPTION)
    actions: List[str] = Field(default=[], description=ACTIONS_DESCRIPTION)


class UnorderedTopQPlannerOutput(BaseModel):
    planner_output: str = Field(default="", description=LOG_DESCRIPTION)
    planner_error: str = Field(default="", description=ERROR_MESSAGE_DESCRIPTION)
    timeout_triggered: bool = Field(
        default=False, description=TIMEOUT_TRIGGERED_DESCRIPTION
    )
    plans: List[Plan] = Field(default=[], description=PLANS_DESCRIPTION)
