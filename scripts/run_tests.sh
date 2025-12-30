#!/bin/bash
# Run tests using Docker

# Navigate to project root
cd "$(dirname "$0")/.."

# Determine Docker Compose command
if command -v docker &> /dev/null && docker compose version &> /dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
elif command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
else
    echo "❌ Docker Compose not found. Please install Docker Compose."
    exit 1
fi

# Determine if sudo is needed for Docker
if docker version > /dev/null 2>&1; then
    SUDO=""
else
    SUDO="sudo"
fi

# Load .env file if it exists
if [ -f .env ]; then
    echo "📄 Loading environment from .env..."
    export $(grep -v '^#' .env | xargs)
fi

echo "🚀 Building and running tests..."

# Check for recording flag
RECORD_FLAG=""
if [[ "$*" == *"--record"* ]]; then
    echo "📹 Recording test run to test_results/bags..."
    RECORD_FLAG="--profile record"
fi

# Check if user wants to run live tests
if [ -z "$SPECKLE_TOKEN" ]; then
    echo "ℹ️  SPECKLE_TOKEN not set. Live tests will be skipped."
else
    echo "✅ SPECKLE_TOKEN detected. Live tests will be attempted."
fi

# Pass environment variables to docker-compose
SPECKLE_TOKEN=${SPECKLE_TOKEN} \
TEST_STREAM_ID=${TEST_STREAM_ID} \
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp} \
$SUDO $DOCKER_COMPOSE -f docker-compose.test.yml $RECORD_FLAG up --build --abort-on-container-exit --exit-code-from test
