package main

import (
	"fmt"
	"time"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

type Simulator struct {
	drone     *Drone
	camera    *Camera
	renderer  *Renderer
	input     *InputHandler
	lastTime  time.Time
}

func NewSimulator() *Simulator {
	drone := NewDrone()
	camera := NewCamera()
	// Initialize camera to look at drone
	camera.Target = drone.Position
	
	return &Simulator{
		drone:    drone,
		camera:   camera,
		renderer: NewRenderer(),
		input:    NewInputHandler(),
		lastTime: time.Now(),
	}
}

func (s *Simulator) Run(window *glfw.Window) {
	s.input.SetupCallbacks(window)
	
	fmt.Println("=== REALISTIC DRONE FLIGHT SIMULATOR ===")
	fmt.Printf("Aircraft: Consumer Quadcopter (%.0fg)\n", s.drone.Mass*1000)
	fmt.Printf("Battery: %.0f%% | Status: DISARMED\n", s.drone.BatteryPercent)
	fmt.Println()
	fmt.Println("SAFETY PROCEDURES:")
	fmt.Println("  1. Hold SPACE + ENTER for 2 seconds to ARM")
	fmt.Println("  2. ESC - Emergency DISARM")
	fmt.Println("  3. H - Emergency hover")
	fmt.Println()
	fmt.Println("FLIGHT CONTROLS:")
	fmt.Println("  W/S - Throttle up/down (gradual)")
	fmt.Println("  A/D - Yaw left/right")
	fmt.Println("  Q/E - Roll left/right")
	fmt.Println("  Up/Down - Pitch forward/back")
	fmt.Println("  Z - Zero throttle  X - Hover throttle")
	fmt.Println()
	fmt.Println("FLIGHT MODES:")
	fmt.Println("  1 - Manual  2 - Altitude Hold  3 - Hover")
	fmt.Println()
	fmt.Println("CAMERA MODES:")
	fmt.Println("  C - Cycle camera modes (Follow → Top-Down → FPV)")
	fmt.Println("  FOLLOW: Arrow Keys - Rotate, +/- - Zoom")
	fmt.Println("  TOP-DOWN: +/- - Height adjustment")
	fmt.Println("  FPV: Drone's eye view")
	fmt.Println()
	fmt.Println("FLIGHT ENVELOPE:")
	fmt.Printf("  Max Speed: %.0f m/s | Max Altitude: %.0fm\n", s.drone.MaxSpeed, s.drone.MaxAltitude)
	fmt.Printf("  Max Climb Rate: %.0f m/s | Battery Life: ~25min\n", s.drone.MaxVerticalSpeed)
	fmt.Println()

	frameCount := 0
	telemetryTimer := 0.0
	
	for !window.ShouldClose() {
		currentTime := time.Now()
		dt := currentTime.Sub(s.lastTime).Seconds()
		s.lastTime = currentTime

		// Cap delta time to prevent large jumps
		if dt > 0.016 {
			dt = 0.016
		}

		s.processInput(dt)
		s.update(dt)
		s.render(window)

		// Display telemetry every 2 seconds (like real drone telemetry)
		telemetryTimer += dt
		if telemetryTimer >= 2.0 {
			s.displayTelemetry()
			telemetryTimer = 0.0
		}

		window.SwapBuffers()
		glfw.PollEvents()
		frameCount++
	}
}

func (s *Simulator) displayTelemetry() {
	drone := s.drone
	camera := s.camera
	
	// Clear screen and show current status (like OSD overlay)
	fmt.Printf("\r\033[K") // Clear current line
	
	// Armed status
	status := "DISARMED"
	if drone.IsArmed {
		status = "ARMED"
	}
	
	// Flight mode
	mode := "MANUAL"
	switch drone.FlightMode {
	case FlightModeAltitudeHold:
		mode = "ALT HOLD"
	case FlightModeHover:
		mode = "HOVER"
	}
	
	// Camera mode
	cameraMode := "FOLLOW"
	switch camera.Mode {
	case CameraModeTopDown:
		cameraMode = "TOP-DOWN"
	case CameraModeFPV:
		cameraMode = "FPV"
	}
	
	// Battery warning
	batteryStatus := "OK"
	if drone.BatteryPercent <= drone.CriticalBattery {
		batteryStatus = "CRITICAL!"
	} else if drone.BatteryPercent <= drone.LowBatteryWarning {
		batteryStatus = "LOW"
	}
	
	// Speed calculation
	speed := drone.Velocity.Length()
	horizontalSpeed := Vec3{drone.Velocity.X, 0, drone.Velocity.Z}.Length()
	
	fmt.Printf("TELEMETRY | %s | %s | CAM: %s | Battery: %.1f%% (%s) | Alt: %.1fm | Speed: %.1fm/s | Throttle: %.0f%% | Power: %.0fW",
		status, mode, cameraMode, drone.BatteryPercent, batteryStatus, 
		drone.Position.Y, horizontalSpeed, drone.ThrottlePercent, drone.PowerDraw)
	
	// Warnings
	if drone.Position.Y > drone.MaxAltitude * 0.9 {
		fmt.Print(" | ⚠ ALTITUDE LIMIT")
	}
	if speed > drone.MaxSpeed * 0.9 {
		fmt.Print(" | ⚠ MAX SPEED")
	}
}

func (s *Simulator) processInput(dt float64) {
	s.input.ProcessInput(s.drone, s.camera, dt)
}

func (s *Simulator) update(dt float64) {
	s.drone.Update(dt)
	s.camera.Update(s.drone)
}

func (s *Simulator) render(window *glfw.Window) {
    gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	width, height := window.GetFramebufferSize()
	gl.Viewport(0, 0, int32(width), int32(height))
	
	// Use camera's projection matrix for consistency
	projection := s.camera.GetProjectionMatrix(width, height)
	view := s.camera.GetViewMatrix()

    // Update renderer with camera position for fog/grid
    s.renderer.SetCamera(s.camera.Position)

    // Render ground centered under camera target so it appears infinite
    groundModel := TranslationMat4(Vec3{X: s.camera.Target.X, Y: 0, Z: s.camera.Target.Z})
    s.renderer.SetMatrices(groundModel, view, projection)
    s.renderer.RenderGround()

	// Render drone at its world position
	droneModel := s.drone.GetTransformMatrix()
	s.renderer.SetMatrices(droneModel, view, projection)
	s.renderer.RenderDrone()
}
