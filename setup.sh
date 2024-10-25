#!/bin/bash

# Exit on error
set -e

echo "Starting PX4-Autopilot setup..."

# Setup PX4-Autopilot dependencies
if ! ./PX4-Autopilot/Tools/setup/ubuntu.sh; then
    echo "Error: Failed to run ubuntu setup script"
    exit 1
fi

# Change directory and build SITL
echo "Building PX4 SITL..."
cd PX4-Autopilot/
if ! make px4_sitl; then
    echo "Error: Failed to build PX4 SITL"
    exit 1
fi

# Create and move to monopulse directory
echo "Setting up QGroundControl..."
cd ~/monopulse

# Download and setup QGroundControl
if ! wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage; then
    echo "Error: Failed to download QGroundControl"
    exit 1
fi

chmod +x QGroundControl.AppImage

echo "Setup completed successfully!"
