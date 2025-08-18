# 3D Drone Simulator

A 3D drone simulator written in Go using OpenGL for rendering and realistic physics simulation.

## Features

- Real-time 3D drone physics simulation with gravity, thrust, and drag
- 3D rendering using OpenGL with colored cube drone and ground plane
- Interactive camera system that follows the drone
- Keyboard controls for drone movement and camera manipulation
- Realistic flight dynamics with rotation and angular velocity

## Controls

### Drone Controls
- **W** - Increase thrust (ascend)
- **S** - Decrease thrust (descend)
- **A/D** - Yaw left/right
- **Q/E** - Roll left/right  
- **Up/Down Arrow** - Pitch forward/backward

### Camera Controls
- **Left/Right Arrow** - Rotate camera around drone
- **Page Up/Page Down** - Adjust camera pitch
- **+/-** - Zoom in/out

## Requirements

- Go 1.19 or later
- OpenGL 4.1 compatible graphics card
- GLFW library (automatically installed via go mod)

## Installation & Running

1. Clone or download the project
2. Install dependencies:
   ```bash
   go mod tidy
   ```
3. Run the simulator:
   ```bash
   go run .
   ```

## Architecture

- `main.go` - Entry point and OpenGL initialization
- `drone.go` - Drone physics model and state management
- `math.go` - 3D vector and matrix mathematics utilities
- `renderer.go` - OpenGL rendering system with shaders
- `camera.go` - 3D camera system with orbital controls
- `input.go` - Keyboard input handling
- `simulator.go` - Main simulation loop and coordination

## Physics Model

The drone simulation includes:
- Gravitational force (9.81 m/s²)
- Thrust force in the drone's local up direction
- Air resistance/drag proportional to velocity
- Angular velocity and rotation dynamics
- Ground collision detection

The drone behaves like a simplified quadcopter with realistic flight characteristics.