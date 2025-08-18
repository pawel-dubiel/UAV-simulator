package main

import (
	"github.com/go-gl/glfw/v3.3/glfw"
)

type InputHandler struct {
	keys        map[glfw.Key]bool
	keyPressed  map[glfw.Key]bool  // Single key press detection
	armingTime  float64            // Time holding arm sequence
}

func NewInputHandler() *InputHandler {
	return &InputHandler{
		keys:       make(map[glfw.Key]bool),
		keyPressed: make(map[glfw.Key]bool),
		armingTime: 0.0,
	}
}

func (i *InputHandler) SetupCallbacks(window *glfw.Window) {
	window.SetKeyCallback(func(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
		if action == glfw.Press {
			i.keys[key] = true
			i.keyPressed[key] = true
		} else if action == glfw.Release {
			i.keys[key] = false
		}
	})
}

func (i *InputHandler) IsKeyPressed(key glfw.Key) bool {
	return i.keys[key]
}

func (i *InputHandler) WasKeyPressed(key glfw.Key) bool {
	if i.keyPressed[key] {
		i.keyPressed[key] = false // Reset for next frame
		return true
	}
	return false
}

func (i *InputHandler) ProcessInput(drone *Drone, camera *Camera, dt float64) {
	// Flight control inputs
	throttleInput := 0.0
	torque := Vec3{0, 0, 0}

	// Throttle control (only works when armed - realistic safety)
	if drone.IsArmed {
		if i.IsKeyPressed(glfw.KeyW) {
			throttleInput += 80.0 * dt // Smooth throttle increase
		}
		if i.IsKeyPressed(glfw.KeyS) {
			throttleInput -= 60.0 * dt // Smooth throttle decrease
		}
		
		// Apply throttle change
		newThrottle := drone.ThrottlePercent + throttleInput
		drone.SetThrottle(newThrottle)
	}
	
	// Rotation controls (pilot stick inputs)
	torqueScale := 1.5 // Realistic control authority
	if i.IsKeyPressed(glfw.KeyA) {
		torque.Y += torqueScale // Yaw left
	}
	if i.IsKeyPressed(glfw.KeyD) {
		torque.Y -= torqueScale // Yaw right
	}
	if i.IsKeyPressed(glfw.KeyQ) {
		torque.Z += torqueScale // Roll left
	}
	if i.IsKeyPressed(glfw.KeyE) {
		torque.Z -= torqueScale // Roll right
	}
	if i.IsKeyPressed(glfw.KeyUp) {
		torque.X -= torqueScale // Pitch forward
	}
	if i.IsKeyPressed(glfw.KeyDown) {
		torque.X += torqueScale // Pitch backward
	}

    drone.AddTorque(torque, dt)

	// SAFETY CONTROLS (Essential for realistic drone operation)
	
	// Arming sequence: Hold Space + Enter for 2 seconds (realistic safety procedure)
	if i.IsKeyPressed(glfw.KeySpace) && i.IsKeyPressed(glfw.KeyEnter) {
		i.armingTime += dt
		if i.armingTime >= 2.0 && !drone.IsArmed {
			drone.Arm()
			i.armingTime = 0.0
		}
	} else {
		i.armingTime = 0.0
	}
	
	// Emergency disarm: Press Escape (immediate safety cutoff)
	if i.WasKeyPressed(glfw.KeyEscape) {
		drone.Disarm()
	}
	
	// Flight mode switching (realistic modes)
	if i.WasKeyPressed(glfw.Key1) {
		drone.SetFlightMode(FlightModeManual)
	}
	if i.WasKeyPressed(glfw.Key2) {
		drone.SetFlightMode(FlightModeAltitudeHold)
	}
	if i.WasKeyPressed(glfw.Key3) {
		drone.SetFlightMode(FlightModeHover)
	}
	
	// Emergency procedures
    if i.WasKeyPressed(glfw.KeyH) {
        // Emergency hover at current altitude
        drone.SetFlightMode(FlightModeHover)
        drone.SetThrottle(drone.HoverThrottlePercent()) // Hover throttle
    }
	
	// Quick throttle presets (only when armed)
	if drone.IsArmed {
		if i.WasKeyPressed(glfw.KeyZ) {
			drone.SetThrottle(0.0) // Zero throttle
		}
        if i.WasKeyPressed(glfw.KeyX) {
            drone.SetThrottle(drone.HoverThrottlePercent()) // Hover throttle (~63%)
        }
    }

	// Camera mode switching (realistic pilot views)
	if i.WasKeyPressed(glfw.KeyC) {
		// Cycle through camera modes
		switch camera.Mode {
		case CameraModeFollow:
			camera.SetMode(CameraModeTopDown)
		case CameraModeTopDown:
			camera.SetMode(CameraModeFPV)
		case CameraModeFPV:
			camera.SetMode(CameraModeFollow)
		}
	}
	
	// Top-down view controls (when in top-down mode)
	if camera.Mode == CameraModeTopDown {
		if i.IsKeyPressed(glfw.KeyMinus) {
			camera.AdjustTopDownHeight(20.0 * dt) // Zoom out (higher)
		}
		if i.IsKeyPressed(glfw.KeyEqual) {
			camera.AdjustTopDownHeight(-20.0 * dt) // Zoom in (lower)
		}
	}
	
    // Follow camera controls (when in follow mode)
    if camera.Mode == CameraModeFollow {
        // Allow fine/coarse adjustment with Alt/Shift modifiers
        yawSpeed := 60.0
        pitchSpeed := 30.0
        zoomSpeed := 8.0

        if i.IsKeyPressed(glfw.KeyLeftShift) || i.IsKeyPressed(glfw.KeyRightShift) {
            yawSpeed *= 2.0
            pitchSpeed *= 2.0
            zoomSpeed *= 1.5
        }
        if i.IsKeyPressed(glfw.KeyLeftAlt) || i.IsKeyPressed(glfw.KeyRightAlt) {
            yawSpeed *= 0.5
            pitchSpeed *= 0.5
            zoomSpeed *= 0.7
        }

        if i.IsKeyPressed(glfw.KeyLeft) {
            camera.Yaw -= yawSpeed * dt
        }
        if i.IsKeyPressed(glfw.KeyRight) {
            camera.Yaw += yawSpeed * dt
        }
        if i.IsKeyPressed(glfw.KeyPageUp) {
            camera.Pitch += pitchSpeed * dt
            if camera.Pitch > 80 {
                camera.Pitch = 80
            }
        }
        if i.IsKeyPressed(glfw.KeyPageDown) {
            camera.Pitch -= pitchSpeed * dt
            if camera.Pitch < -80 {
                camera.Pitch = -80
            }
        }
        if i.IsKeyPressed(glfw.KeyMinus) {
            camera.Distance += zoomSpeed * dt
            if camera.Distance > 50.0 {
                camera.Distance = 50.0
            }
        }
        if i.IsKeyPressed(glfw.KeyEqual) {
            camera.Distance -= zoomSpeed * dt
            if camera.Distance < 1.0 {
                camera.Distance = 1.0
            }
        }
    }
}
