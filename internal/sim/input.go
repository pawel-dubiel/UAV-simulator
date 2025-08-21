//go:build !test
// +build !test

package sim

import (
	"github.com/go-gl/glfw/v3.3/glfw"
)

type InputHandler struct {
	keys       map[glfw.Key]bool
	keyPressed map[glfw.Key]bool // Single key press detection
	armingTime float64           // Time holding arm sequence
	// Mouse input for universal camera control
	rotating    bool
	lastX       float64
	lastY       float64
	scrollDelta float64
	mouseDX     float64
	mouseDY     float64
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

	window.SetMouseButtonCallback(func(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
		if button == glfw.MouseButton2 { // Right mouse button to orbit camera
			if action == glfw.Press {
				i.rotating = true
				i.lastX, i.lastY = w.GetCursorPos()
			} else if action == glfw.Release {
				i.rotating = false
			}
		}
	})

	window.SetCursorPosCallback(func(w *glfw.Window, xpos float64, ypos float64) {
		if i.rotating {
			dx := xpos - i.lastX
			dy := ypos - i.lastY
			i.lastX = xpos
			i.lastY = ypos
			i.mouseDX += dx
			i.mouseDY += dy
		}
	})

	window.SetScrollCallback(func(w *glfw.Window, xoff float64, yoff float64) {
		i.scrollDelta += yoff
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
		torque.X -= torqueScale // Pitch forward (unless Alt is held for camera control)
	}
	if i.IsKeyPressed(glfw.KeyDown) {
		torque.X += torqueScale // Pitch backward (unless Alt is held for camera control)
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

	// Engine failure/derate toggles for quick testing (F5..F8 for engines 0..3)
	cycle := func(idx int) {
		if idx < 0 || idx >= len(drone.Engines) {
			return
		}
		e := drone.Engines[idx]
		if !e.Functional || e.Efficiency <= 0.01 {
			// Repair to 100%
			drone.RepairEngine(idx)
			return
		}
		if e.Efficiency >= 0.99 {
			// Derate to 50%
			drone.SetEngineEfficiency(idx, 0.5)
			return
		}
		// Fail engine
		drone.FailEngine(idx)
	}
	if i.WasKeyPressed(glfw.KeyF5) { cycle(0) }
	if i.WasKeyPressed(glfw.KeyF6) { cycle(1) }
	if i.WasKeyPressed(glfw.KeyF7) { cycle(2) }
	if i.WasKeyPressed(glfw.KeyF8) { cycle(3) }
	// F9 repairs all
	if i.WasKeyPressed(glfw.KeyF9) {
		for idx := range drone.Engines {
			drone.RepairEngine(idx)
		}
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
		// Remove PageUp/PageDown dependency; use mouse for pitch (see below)
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

	// Mouse-based universal camera control (Follow mode)
	if camera.Mode == CameraModeFollow && i.rotating {
		// Convert pixel deltas to degrees
		yawDelta := i.mouseDX * 0.2
		pitchDelta := -i.mouseDY * 0.2
		camera.Yaw += yawDelta
		camera.Pitch += pitchDelta
		// Reset consumed deltas
		i.mouseDX = 0
		i.mouseDY = 0
		if camera.Pitch > 80 {
			camera.Pitch = 80
		}
		if camera.Pitch < -80 {
			camera.Pitch = -80
		}
	}

	// Mouse/trackpad scroll for zoom/height
	if i.scrollDelta != 0 {
		if camera.Mode == CameraModeFollow {
			camera.Distance -= i.scrollDelta * 1.0 // zoom speed per tick
			if camera.Distance < 1.0 {
				camera.Distance = 1.0
			}
			if camera.Distance > 50.0 {
				camera.Distance = 50.0
			}
		} else if camera.Mode == CameraModeTopDown {
			camera.AdjustTopDownHeight(-i.scrollDelta * 2.0)
		}
		i.scrollDelta = 0
	}
}
