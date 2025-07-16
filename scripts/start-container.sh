#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/../.devcontainer/docker-compose.yaml"

echo "Starting runner service..."
docker compose -f "$COMPOSE_FILE" up -d runner

echo "Attaching to runner shell..."
docker compose -f "$COMPOSE_FILE" exec runner bash