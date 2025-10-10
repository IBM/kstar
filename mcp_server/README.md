# K* MCP Server

K* MCP Server provides a containerized deployment of Top-K and Top-Q planners from the [KStar repository](https://github.com/IBM/kstar) as Model Checking Problem (MCP) tools. This allows for easy deployment and execution of these planners within a Docker container.

## Prerequisites
*[Docker](https://docs.docker.com/get-docker/) installed on your system.

## Building the Docker Image

To build the Docker image, navigate to the project root directory (where the `pyproject.toml` is located) and execute the following command:

```bash
docker build -f mcp_server/Dockerfile.mcp -t kstar-planner:v0.0 .
```

*   `-f mcp_server/Dockerfile.mcp`: Specifies the Dockerfile to use for building the image.
*   `-t kstar-planner:v0.0`:  Tags the image as `kstar-planner` with the version `v0.0`.  You can replace `v0.0` with your desired version tag.
*   `.`:  Indicates that the build context is the current directory.
   
## Running a Container

Once the image is built, you can run a K* MCP Server instance using the following command:

```bash
docker run -p 8000:8000 --name kstar-planner -d kstar-planner:v0.0
```

*   `-p 8000:8000`:  Maps port 8000 on your host machine to port 8000 inside the container.  This allows you to access the server from your host.
*   `--name kstar-planner`: Assigns the name `kstar-planner` to the container, making it easier to manage.
*   `-d`: Runs the container in detached mode (in the background).
*   `kstar-planner:v0.0`: Specifies the image to use for creating the container.
   
## Accessing the Server:

After running the container, you should be able to access the K* MCP Server at `http://localhost:8000` (or the appropriate host/IP address if not running locally).