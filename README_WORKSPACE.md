# Sweety ROS2 AMR Workspace

This workspace is configured for building and managing ROS2 packages for the Sweety Autonomous Mobile Robot.

## Workspace Structure

```
sweety-ros2-amr/
├── src/                  # Source code for ROS2 packages
├── build/               # Build artifacts (auto-generated)
├── install/             # Installed packages (auto-generated)
├── log/                 # Build logs (auto-generated)
├── colcon_ws.sh         # Workspace management script
└── README_WORKSPACE.md  # This file
```

## Prerequisites

- ROS2 (tested with Humble/Iron)
- colcon build tool
- Python 3.8+
- git

## Quick Start

### 1. Initialize the Workspace

```bash
./colcon_ws.sh setup
```

This will create the necessary directory structure and verify your ROS2 installation.

### 2. Add Packages

Clone or create your ROS2 packages in the `src/` directory:

```bash
cd src/
git clone <package-repo-url>
```

### 3. Build the Workspace

```bash
./colcon_ws.sh build
```

### 4. Source the Workspace

To use the packages in your current shell session:

```bash
source ./colcon_ws.sh source
```

Or add this to your `.bashrc` for automatic sourcing:

```bash
source ~/path/to/sweety-ros2-amr/install/setup.bash
```

## Available Commands

- `./colcon_ws.sh setup` - Initialize workspace directories
- `./colcon_ws.sh build` - Build all packages
- `./colcon_ws.sh source` - Source workspace (use with `source` command)
- `./colcon_ws.sh clean` - Clean build artifacts
- `./colcon_ws.sh check` - Verify ROS2 and colcon installation
- `./colcon_ws.sh help` - Show help message

## Building Specific Packages

To build only specific packages:

```bash
cd <workspace-root>
colcon build --packages-select <package-name>
```

## Troubleshooting

### ROS2 not found

Make sure ROS2 is installed and sourced:

```bash
source /opt/ros/<distro>/setup.bash
```

### colcon not found

Install colcon:

```bash
sudo apt install python3-colcon-common-extensions
```

### Build failures

1. Check the logs in the `log/` directory
2. Ensure all dependencies are installed
3. Run `./colcon_ws.sh clean` and rebuild

## Package Development

When adding new packages, ensure they follow ROS2 standards:

- C++ packages: Use `ament_cmake`
- Python packages: Use `ament_python`
- Always include proper `package.xml` and `CMakeLists.txt` or `setup.py`

## References

- [ROS2 Documentation](https://docs.ros.org/)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ROS2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
