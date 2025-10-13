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
                {"domain": domain, "problem": problem},
            )
            assert payload is not None
            assert len(payload.structured_content["plans"]) == 1
            optimal_plan = payload.structured_content["plans"][0]
            assert len(optimal_plan["actions"]) == 4
            assert optimal_plan["cost"] == 4
