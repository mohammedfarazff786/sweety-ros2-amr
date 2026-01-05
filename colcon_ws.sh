#!/bin/bash

# ROS2 Sweety AMR Workspace Setup Script
# This script initializes and manages the ROS2 workspace

set -e

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Functions
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

check_ros2_installation() {
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 is not installed or not in PATH"
        return 1
    fi
    print_info "ROS2 found: $(ros2 --version)"
    return 0
}

check_colcon_installation() {
    if ! command -v colcon &> /dev/null; then
        print_error "colcon is not installed or not in PATH"
        return 1
    fi
    print_info "colcon found: $(colcon --version)"
    return 0
}

setup_workspace() {
    print_info "Setting up ROS2 workspace at $WS_DIR"
    
    # Create necessary directories if they don't exist
    mkdir -p "$WS_DIR/src"
    mkdir -p "$WS_DIR/build"
    mkdir -p "$WS_DIR/install"
    mkdir -p "$WS_DIR/log"
    
    print_info "Workspace directories created/verified"
}

build_workspace() {
    print_info "Building workspace..."
    
    if [ ! -d "$WS_DIR/src" ]; then
        print_error "Source directory not found. Run 'setup' first."
        return 1
    fi
    
    cd "$WS_DIR"
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        print_info "Workspace built successfully"
    else
        print_error "Build failed"
        return 1
    fi
}

source_workspace() {
    print_info "Sourcing workspace setup..."
    
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
        print_info "Workspace sourced. ROS2_DISTRO: $ROS_DISTRO"
    else
        print_error "Setup file not found. Build the workspace first."
        return 1
    fi
}

clean_workspace() {
    print_warning "Cleaning build artifacts..."
    
    rm -rf "$WS_DIR/build"
    rm -rf "$WS_DIR/install"
    rm -rf "$WS_DIR/log"
    
    mkdir -p "$WS_DIR/build"
    mkdir -p "$WS_DIR/install"
    mkdir -p "$WS_DIR/log"
    
    print_info "Workspace cleaned"
}

show_usage() {
    cat << EOF
ROS2 Sweety AMR Workspace Management Script

Usage: $0 [COMMAND]

Commands:
    setup       - Initialize workspace directory structure
    build       - Build all packages in the workspace
    source      - Source the workspace setup (only works with 'source' command)
    clean       - Clean build artifacts and regenerate directories
    check       - Check ROS2 and colcon installation
    help        - Show this help message

Examples:
    $0 setup        # Initialize workspace
    $0 build        # Build workspace
    source $0 source # Source workspace (must use 'source' command)
    $0 clean        # Clean workspace
    $0 check        # Check dependencies

Note: To properly source the workspace in your current shell, use:
    source ./colcon_ws.sh source

EOF
}

# Main script logic
COMMAND="${1:-help}"

case "$COMMAND" in
    setup)
        check_ros2_installation || exit 1
        check_colcon_installation || exit 1
        setup_workspace
        ;;
    build)
        check_ros2_installation || exit 1
        check_colcon_installation || exit 1
        build_workspace
        ;;
    source)
        source_workspace
        ;;
    clean)
        clean_workspace
        ;;
    check)
        check_ros2_installation
        check_colcon_installation
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        show_usage
        exit 1
        ;;
esac
