import os
import pytest
from mcp_server.tests.data.pddl.pddl_example import domain, problem
from fastmcp import Client
from fastmcp.client.transports import StreamableHttpTransport


def get_client(server_url) -> Client:
    transport = StreamableHttpTransport(url=server_url)
    return Client(transport)

class TestMcpContainer:
    @pytest.mark.skipif(
        "MCP_SERVER_URL" not in os.environ, reason="Requires MCP_SERVER_URL to be set"
    )
    @pytest.mark.asyncio
    async def test_planner_tool_t(self) -> None:
        client = get_client(os.getenv("MCP_SERVER_URL", "http://localhost:8000/mcp"))

        async with client:
            tool_list = await client.list_tools()
            assert tool_list is not None

            payload = await client.call_tool(
                "KstarPlannerUnorderedTopQ",
                {"domain": domain, "problem": problem, "quality_bound": 20.0, "num_plans": 1000},
            )

            assertion_status_payload = payload is not None
            assert assertion_status_payload, "Payload is empty."
            if assertion_status_payload:
                print("Payload is not empty.")

            assertion_status_plans = len(payload.structured_content["plans"]) == 1000
            assert assertion_status_plans, "The number of plans returned by KstarPlannerUnorderedTopQ is not 1000."
            if assertion_status_payload:
                print("The number of plans returned by KstarPlannerUnorderedTopQ is 1000.")

            optimal_plan = payload.structured_content["plans"][0]

            assertion_status_actions = len(optimal_plan["actions"]) == 4
            assert assertion_status_actions, "The number of actions found in the optimal plan returned by KstarPlannerUnorderedTopQ is not 4."
            if assertion_status_actions:
                print("Four actions are found in the optimal plan returned by KstarPlannerUnorderedTopQ.")
            
            assertion_status_cost = optimal_plan["cost"] == 4
            assert assertion_status_cost, "The cost of the optimal plan returned by KstarPlannerUnorderedTopQ is not 4."
            if assertion_status_cost:
                print("The cost of the optimal plan returned by KstarPlannerUnorderedTopQ is 4.")
