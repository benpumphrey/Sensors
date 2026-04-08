#!/bin/bash
#
# This script installs project dependencies for ROS 2 Humble.
# It automatically detects the OS and uses the appropriate package manager.
#

set -e # Exit immediately if a command exits with a non-zero status.

# --- OS Detection ---
OS_NAME=$(uname -s)

if [ "$OS_NAME" == "Linux" ]; then
    echo "🐧 Detected Linux. Using 'rosdep' for dependency installation."

    # Check if rosdep is initialized, and initialize it if not.
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "rosdep not found or not initialized. Initializing..."
        sudo apt-get update
        sudo apt-get install -y python3-rosdep
        sudo rosdep init
    fi

    echo "Updating rosdep..."
    rosdep update

    echo "Installing dependencies from all 'package.xml' files in 'src/'..."
    rosdep install --from-paths src --ignore-src -r -y

    echo "✅ Linux dependencies installed successfully."

elif [ "$OS_NAME" == "Darwin" ]; then
    echo "🍏 Detected macOS. Dependencies are managed by Conda."

    if ! command -v conda &> /dev/null; then
        echo "❌ Conda is not installed or not in your PATH." >&2
        echo "Please install Conda (Miniforge/Mambaforge is recommended) and try again." >&2
        exit 1
    fi

    echo "Please ensure your Conda environment ('ttt_env') is activated and has the required robostack packages."
    echo "For team collaboration, it is highly recommended to create an 'environment.yml' file by running:"
    echo "conda env export --from-history > environment.yml"
    echo "New users can then create the environment with: 'conda env create -f environment.yml'"
    echo "✅ macOS dependency check complete. Please manage your Conda environment as described above."

else
    echo "❌ Unsupported OS: $OS_NAME" >&2
    exit 1
fi