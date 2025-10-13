from fastmcp import FastMCP
from pydantic import Field

from helpers.planner_helper import get_plan_unordered_topq
from data.descriptions.tool_descriptions import (
    NUM_PLANS_DESCRIPTION,
    PDDL_DOMAIN_STRING_DESCRIPTION,
    PDDL_PROBLEM_STRING_DESCRIPTION,
    QUALITY_BOUND_DESCRIPTION,
    TIMEOUT_DESCRIPTION,
    UNORDERED_TOP_Q_PLANNER_DESCRIPTION,
)
from data_models.tool_data_models import UnorderedTopQPlannerOutput


def get_fastmcp_server(name: str) -> FastMCP:
    mcp = FastMCP(name)

    @mcp.tool(
        name="KstarPlannerUnorderedTopQ",
        description=UNORDERED_TOP_Q_PLANNER_DESCRIPTION,
    )
    def planner_unordered_topq(
        domain: str = Field(description=PDDL_DOMAIN_STRING_DESCRIPTION),
        problem: str = Field(description=PDDL_PROBLEM_STRING_DESCRIPTION),
        timeout: int = Field(
            default=600,
            description=TIMEOUT_DESCRIPTION,
        ),
        quality_bound: float = Field(
            default=1.0,
            description=QUALITY_BOUND_DESCRIPTION,
        ),
        num_plans: int = Field(
            default=10,
            description=NUM_PLANS_DESCRIPTION,
        ),
    ) -> UnorderedTopQPlannerOutput:
        output_dict = get_plan_unordered_topq(
            domain=domain,
            problem=problem,
            timeout=timeout,
            quality_bound=quality_bound,
            num_plans=num_plans,
        )

        return UnorderedTopQPlannerOutput.model_validate(output_dict)

    return mcp


mcp = get_fastmcp_server("K*")

if __name__ == "__main__":
    mcp.run(transport="http", port=8000)
