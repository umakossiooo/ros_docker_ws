## ROS Docker Workspace - Quick Start with Docker Compose

This project uses Docker Compose to simplify the setup and management of ROS 2 Jazzy development containers.

### Prerequisites

- Docker installed
- Docker Compose installed

### Usage

1. **Build the Docker image:**
	```bash
	docker-compose build
	```

2. **Start the container:**
	```bash
	docker-compose up
	```
	This will launch the container with all required volumes and environment variables.

3. **Stop and remove the container:**
	```bash
	docker-compose down
	```

### Notes

- The container mounts your ROS project folders as volumes, so changes in your local files are reflected inside the container.
- If you update the `Dockerfile` (e.g., add new packages), rebuild the image with `docker-compose build`.
- You do not need to manually create or delete containers; Docker Compose handles this for you.

### Troubleshooting

- If you have display issues, ensure your `DISPLAY` environment variable is set correctly on your host machine.
- For advanced configuration, edit the `docker-compose.yml` file.

---
For more details, see the `docker-compose.yml` in the project root.
