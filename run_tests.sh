#!/bin/bash
# Run tests using Docker

cd "$(dirname "$0")"

echo "🚀 Building and running tests..."

# Check if user wants to run live tests
if [ -z "$SPECKLE_TOKEN" ]; then
    echo "ℹ️  SPECKLE_TOKEN not set. Live tests will be skipped."
else
    echo "✅ SPECKLE_TOKEN detected. Live tests will be attempted."
fi

# Pass environment variables to docker-compose
SPECKLE_TOKEN=${SPECKLE_TOKEN} \
TEST_STREAM_ID=${TEST_STREAM_ID} \
docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from test
