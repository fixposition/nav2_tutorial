#!/bin/bash
docker compose -f ~/dev/nav2_tutorial/.devcontainer/docker-compose.yaml down
docker compose -f ~/dev/nav2_tutorial/.devcontainer/docker-compose.yaml up --build -d
