# Pushing Data Collection System

This system simulates a robot pushing a cylinder to various goal positions and collects the trajectory data. It uses MuJoCo for physics simulation and implements a simple pushing controller.

## System Components

1. **PushEnvironment** (`push_environment.hpp`)
   - Manages the MuJoCo simulation
   - Handles robot and cylinder positioning
   - Implements goal iteration using a grid-based sampling
   - Provides state/control space interfaces

2. **PushController** (`push_controller.hpp`)
   - Base class for push controllers
   - `SimplePushController`: Implements direct pushing towards goal

3. **Interface** (`interface.cpp`)
   - Main execution file
   - Collects and saves trajectory data
   - Manages the interaction between environment and controller

## Configuration

Configuration is done through `examples/tasks/push_plan.yaml`:
- `data_path`: Directory where trajectory data will be saved
- `discretization`: Grid size for sampling goal positions (in meters)
- `random_seed`: For reproducible experiments
- `visualize`: Enable/disable MuJoCo visualization
- `xml_path`: Path to MuJoCo environment file

## Data Collection

For each goal position, the system:
1. Positions the robot behind the cylinder
2. Attempts to push the cylinder to the goal
3. Records state and control trajectories
4. Saves data if successful or timeout reached

### Saved Data Format

Each trajectory is saved in a numbered folder containing:
- `trajectory.txt`: State space trajectory
- `control_trajectory.txt`: Control inputs used
- `metadata.txt`: Contains:
  - XML path used
  - Goal state
  - Success status
  - Number of steps taken

## Usage

1. Configure parameters in `push_plan.yaml`
2. Build the project
3. Run the interface executable:
   ```bash
   ./bin/examples/tamp/interface
   ```

The system will:
- Create a grid of goal positions based on the discretization
- Attempt to reach each goal
- Save all trajectories
- Display overall success rate

## Notes

- The system uses a simple pushing controller that moves directly toward the goal
- Maximum 250 steps per goal attempt
- Goals are sampled in a grid pattern within the environment bounds
- The environment accounts for cylinder and robot radii when sampling goals