#!/bin/bash
# Docker entrypoint script for ROS Speckle Bridge

set -e

# Source ROS setup
source /opt/ros/${ROS_DISTRO}/setup.bash
source /workspace/install/setup.bash

# Validate SPECKLE_TOKEN if not using cached data
if [ -z "$SPECKLE_TOKEN" ]; then
    echo "WARNING: SPECKLE_TOKEN not set. Will attempt to use cached data if available."
    echo "To fetch fresh data, set SPECKLE_TOKEN environment variable."
fi

# Execute the command
exec "$@"
