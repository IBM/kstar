import ast
from fastmcp import Client
from fastmcp.client.transports import StreamableHttpTransport


def unwrap_tool_result(resp):
    """
    Safely unwraps the content from a FastMCP tool call result object.
    """
    if hasattr(resp, "content") and resp.content:
        # The content is a list containing a single content object
        content_object = resp.content[0]
        # It could be JSON or plain text
        if hasattr(content_object, "json"):
            return content_object.json
        if hasattr(content_object, "text"):
            try:
                # Use ast.literal_eval for safely evaluating a string containing a Python literal
                return ast.literal_eval(content_object.text)
            except (ValueError, SyntaxError):
                # If it's not a literal, return the raw text
                return content_object.text
    return resp


def get_client(server_url) -> Client:
    transport = StreamableHttpTransport(url=server_url)
    return Client(transport)
