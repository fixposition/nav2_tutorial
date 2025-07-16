#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/../.devcontainer/docker-compose.yaml"

echo "Showing logs for runner service..."
docker compose -f "$COMPOSE_FILE" logs -f runner
