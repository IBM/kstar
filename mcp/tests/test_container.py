import os
import pytest
from mcp.tests.helpers.mcp_client_helper import get_client
from mcp.tests.data.pddl.pddl_example import domain, problem


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
