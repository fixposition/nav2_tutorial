#!/bin/bash

# Path to the docker-compose.yaml
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/../.devcontainer/docker-compose.yaml"

# Stop and remove all running containers first
docker compose -f "$COMPOSE_FILE" down

# Build the image for the builder service
docker compose -f "$COMPOSE_FILE" build builder
