from fastmcp import Client
from fastmcp.client.transports import StreamableHttpTransport


def get_client(server_url) -> Client:
    transport = StreamableHttpTransport(url=server_url)
    return Client(transport)
