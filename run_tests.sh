#!/bin/bash
# Run tests using Docker

cd "$(dirname "$0")"

echo "Building and running tests..."
docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from test
