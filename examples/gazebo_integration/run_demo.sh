#!/bin/bash
# Quick start script for Gazebo integration demo

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS Speckle Bridge - Gazebo Harmonic Integration ===${NC}\n"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if running as root
is_root() {
    [ "$(id -u)" -eq 0 ]
}

# Function to check if docker works without sudo
docker_works_without_sudo() {
    docker ps >/dev/null 2>&1
}

# Determine docker command
if is_root; then
    DOCKER_CMD="docker"
    echo -e "${YELLOW}Running as root, using docker directly${NC}"
elif docker_works_without_sudo; then
    DOCKER_CMD="docker"
    echo -e "${GREEN}Docker works without sudo${NC}"
else
    DOCKER_CMD="sudo docker"
    echo -e "${YELLOW}Docker requires sudo, using sudo docker${NC}"
fi

echo -e "${GREEN}Running in headless mode (no GUI)${NC}"
echo -e "${GREEN}Use RViz from another machine to visualize${NC}\n"

# Check if .env exists
if [ ! -f .env ]; then
    echo -e "${YELLOW}No .env file found. Creating template...${NC}"
    cat > .env << 'EOF'
# Speckle Configuration
SPECKLE_TOKEN=your_speckle_token_here
SPECKLE_STREAM_ID=your_stream_id_here

# ROS Configuration
ROS_DOMAIN_ID=0

# Gazebo Configuration
GZ_PARTITION=default
EOF
    echo -e "${RED}Please edit .env with your Speckle credentials${NC}"
    exit 1
fi

# Source environment variables
source .env

# Verify required variables
if [ -z "$SPECKLE_TOKEN" ] || [ "$SPECKLE_TOKEN" = "your_speckle_token_here" ]; then
    echo -e "${RED}ERROR: SPECKLE_TOKEN not set in .env${NC}"
    exit 1
fi

if [ -z "$SPECKLE_STREAM_ID" ] || [ "$SPECKLE_STREAM_ID" = "your_stream_id_here" ]; then
    echo -e "${RED}ERROR: SPECKLE_STREAM_ID not set in .env${NC}"
    exit 1
fi

echo -e "${GREEN}Configuration validated${NC}"
echo -e "  Stream ID: ${SPECKLE_STREAM_ID}"
echo -e "  ROS Domain: ${ROS_DOMAIN_ID}"
echo -e "  Mode: Headless (CycloneDDS)\n"

echo -e "${GREEN}Starting services...${NC}\n"

# Run docker compose
$DOCKER_CMD compose up --build --no-deps --force-recreate
