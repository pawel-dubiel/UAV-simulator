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
    ui        *UIRenderer
    lastDt    float64
    fps       float64
    fpsHistory []float64
    fpsIdx     int
    uiVisible  bool
}

func NewSimulator() *Simulator {
	drone := NewDrone()
	camera := NewCamera()
	// Initialize camera to look at drone
	camera.Target = drone.Position
	
    return &Simulator{
        drone:      drone,
        camera:     camera,
        renderer:   NewRenderer(),
        input:      NewInputHandler(),
        lastTime:   time.Now(),
        ui:         NewUIRenderer(),
        fpsHistory: make([]float64, 120),
        fpsIdx:     0,
        uiVisible:  true,
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

        s.lastDt = dt
        if dt > 0 {
            // Smooth FPS estimate
            currentFPS := 1.0 / dt
            s.fps = s.fps*0.9 + currentFPS*0.1
        }

        // Record FPS history (ring buffer)
        s.fpsHistory[s.fpsIdx] = s.fps
        s.fpsIdx = (s.fpsIdx + 1) % len(s.fpsHistory)

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
    // Toggle HUD visibility
    if s.input.WasKeyPressed(glfw.KeyF1) {
        s.uiVisible = !s.uiVisible
    }
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

    // Draw UI overlay panel with telemetry on top of 3D
    if s.uiVisible {
        s.renderUI(width, height)
    }
}

func (s *Simulator) renderUI(width, height int) {
    panelWidth := 360 // pixels
    scaleHeader := 4
    scaleBody := 2
    lineHeight := 8 * scaleBody // 7px font + spacing
    x := 12
    y := 16
    // Draw UI on top of depth by disabling depth test temporarily
    gl.Disable(gl.DEPTH_TEST)
    // Clip all HUD content to the panel bounds
    gl.Enable(gl.SCISSOR_TEST)
    gl.Scissor(0, 0, int32(panelWidth), int32(height))
    s.ui.Begin(width, height)
    // Panel background
    s.ui.AddRect(0, 0, panelWidth, height, Color{0, 0, 0, 0.45})

    // Header
    s.ui.DrawText(x, y, "DRONE STATUS", scaleHeader, Color{1, 1, 1, 1}); y += lineHeight * 2

    // Status strings
    status := "DISARM"
    if s.drone.IsArmed { status = "ARM" }

    mode := "MAN"
    switch s.drone.FlightMode {
    case FlightModeAltitudeHold:
        mode = "ALT"
    case FlightModeHover:
        mode = "HOV"
    }

    cam := "FOL"
    switch s.camera.Mode {
    case CameraModeTopDown:
        cam = "TOP"
    case CameraModeFPV:
        cam = "FPV"
    }

    // Battery status
    batt := "OK"
    if s.drone.BatteryPercent <= s.drone.CriticalBattery { batt = "CRIT" } else if s.drone.BatteryPercent <= s.drone.LowBatteryWarning { batt = "LOW" }

    // Numbers
    throttle := int(s.drone.ThrottlePercent + 0.5)
    power := int(s.drone.PowerDraw + 0.5)
    alt := int(s.drone.Position.Y + 0.5)
    speed := int((Vec3{X: s.drone.Velocity.X, Y: 0, Z: s.drone.Velocity.Z}.Length()) + 0.5)
    batPct := int(s.drone.BatteryPercent + 0.5)
    vspd := int(s.drone.Velocity.Y + 0.5)
    fps := int(s.fps + 0.5)
    ms := int(s.lastDt*1000.0 + 0.5)

    // Lines
    s.ui.DrawText(x, y, "STAT "+status+"   MODE "+mode+"   CAM "+cam, scaleBody, Color{0.9, 0.95, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "SIM FPS "+itoa(fps)+" DT "+itoa(ms)+"MS", scaleBody, Color{0.9, 1, 0.9, 1}); y += lineHeight
    // FPS graph just below
    gx := x
    gy := y
    gw := panelWidth - x - 10
    gh := 48
    // background
    s.ui.AddRect(gx, gy, gw, gh, Color{0, 0, 0, 0.25})
    // draw bars right-to-left
    bars := gw
    if bars > len(s.fpsHistory) { bars = len(s.fpsHistory) }
    for i := 0; i < bars; i++ {
        idx := (s.fpsIdx - 1 - i + len(s.fpsHistory)) % len(s.fpsHistory)
        v := s.fpsHistory[idx]
        if v < 0 { v = 0 }
        if v > 120 { v = 120 }
        h := int((v/120.0)*float64(gh) + 0.5)
        if h < 1 { h = 1 }
        s.ui.AddRect(gx+gw-1-i, gy+gh-h, 1, h, Color{0.2, 0.9, 0.4, 0.9})
    }
    y += gh + 6
    // Battery with color coding
    batColor := Color{0.8, 1, 0.8, 1}
    if s.drone.BatteryPercent <= s.drone.CriticalBattery {
        batColor = Color{1.0, 0.35, 0.35, 1}
    } else if s.drone.BatteryPercent <= s.drone.LowBatteryWarning {
        batColor = Color{1.0, 0.8, 0.3, 1}
    }
    s.ui.DrawText(x, y, "BAT "+itoa(batPct)+"%  "+batt, scaleBody, batColor); y += lineHeight
    s.ui.DrawText(x, y, "THR "+itoa(throttle)+"%   PWR "+itoa(power)+"W", scaleBody, Color{1, 0.95, 0.8, 1}); y += lineHeight
    s.ui.DrawText(x, y, "ALT "+itoa(alt)+"   HSPD "+itoa(speed)+"   VSPD "+itoa(vspd), scaleBody, Color{1, 1, 1, 1}); y += lineHeight

    y += lineHeight // spacer
    s.ui.DrawText(x, y, "POS", scaleBody, Color{0.7, 0.9, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "X "+itoa(int(s.drone.Position.X+0.5)), scaleBody, Color{0.85, 0.9, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "Y "+itoa(int(s.drone.Position.Y+0.5)), scaleBody, Color{0.85, 0.9, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "Z "+itoa(int(s.drone.Position.Z+0.5)), scaleBody, Color{0.85, 0.9, 1, 1}); y += lineHeight

    y += lineHeight // spacer
    s.ui.DrawText(x, y, "ROT", scaleBody, Color{0.7, 0.9, 1, 1}); y += lineHeight
    deg := func(rad float64) int { return int(rad*180.0/3.14159265 + 0.5) }
    s.ui.DrawText(x, y, "P "+itoa(deg(s.drone.Rotation.X)), scaleBody, Color{0.9, 0.85, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "Y "+itoa(deg(s.drone.Rotation.Y)), scaleBody, Color{0.9, 0.85, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "R "+itoa(deg(s.drone.Rotation.Z)), scaleBody, Color{0.9, 0.85, 1, 1}); y += lineHeight

    // Extra telemetry
    // Air density (RHO), hover throttle approximation, power caps
    s.ui.DrawText(x, y, "RHO "+fmt2(s.drone.AirDensity), scaleBody, Color{0.9, 1, 1, 1}); y += lineHeight
    hov := int(s.drone.HoverThrottlePercent()+0.5)
    s.ui.DrawText(x, y, "TH HOV "+itoa(hov)+"%", scaleBody, Color{1, 0.95, 0.9, 1}); y += lineHeight
    s.ui.DrawText(x, y, "PWR MAX "+itoa(int(s.drone.MaxPower+0.5))+"W  HOV "+itoa(int(s.drone.HoverPower+0.5))+"W", scaleBody, Color{1, 0.95, 0.9, 1}); y += lineHeight
    wx := int(s.drone.WindVelocity.X + 0.5)
    wy := int(s.drone.WindVelocity.Y + 0.5)
    wz := int(s.drone.WindVelocity.Z + 0.5)
    s.ui.DrawText(x, y, "WIND X "+itoa(wx)+" Y "+itoa(wy)+" Z "+itoa(wz), scaleBody, Color{0.85, 1, 0.95, 1}); y += lineHeight
    // Limits and targets
    s.ui.DrawText(x, y, "LIM H "+itoa(int(s.drone.MaxSpeed+0.5))+"  V "+itoa(int(s.drone.MaxVerticalSpeed+0.5)), scaleBody, Color{1, 0.9, 1, 1}); y += lineHeight
    s.ui.DrawText(x, y, "LIM ALT "+itoa(int(s.drone.MaxAltitude+0.5)), scaleBody, Color{1, 0.9, 1, 1}); y += lineHeight
    if s.drone.FlightMode == FlightModeAltitudeHold || s.drone.FlightMode == FlightModeHover {
        s.ui.DrawText(x, y, "ALT TGT "+itoa(int(s.drone.AltitudeHold+0.5)), scaleBody, Color{0.95, 1, 0.95, 1}); y += lineHeight
    }
    // Ground contact
    ground := "NO"
    if s.drone.OnGround { ground = "YES" }
    s.ui.DrawText(x, y, "ONGROUND "+ground, scaleBody, Color{0.95, 0.95, 1, 1}); y += lineHeight
    // Motor RPM (rounded)
    r0 := int(s.drone.PropSpeeds[0] + 0.5)
    r1 := int(s.drone.PropSpeeds[1] + 0.5)
    r2 := int(s.drone.PropSpeeds[2] + 0.5)
    r3 := int(s.drone.PropSpeeds[3] + 0.5)
    s.ui.DrawText(x, y, "PROP 0/1 "+itoa(r0)+" "+itoa(r1), scaleBody, Color{1, 0.9, 0.9, 1}); y += lineHeight
    s.ui.DrawText(x, y, "PROP 2/3 "+itoa(r2)+" "+itoa(r3), scaleBody, Color{1, 0.9, 0.9, 1}); y += lineHeight
    // Motor temps
    t0 := int(s.drone.MotorTempC[0] + 0.5)
    t1 := int(s.drone.MotorTempC[1] + 0.5)
    t2 := int(s.drone.MotorTempC[2] + 0.5)
    t3 := int(s.drone.MotorTempC[3] + 0.5)
    s.ui.DrawText(x, y, "TEMP 0/1 "+itoa(t0)+"C "+itoa(t1)+"C", scaleBody, Color{1, 0.85, 0.85, 1}); y += lineHeight
    s.ui.DrawText(x, y, "TEMP 2/3 "+itoa(t2)+"C "+itoa(t3)+"C", scaleBody, Color{1, 0.85, 0.85, 1}); y += lineHeight
    // Camera parameters
    switch s.camera.Mode {
    case CameraModeFollow:
        s.ui.DrawText(x, y, "CAMF Y "+itoa(int(s.camera.Yaw+0.5))+" P "+itoa(int(s.camera.Pitch+0.5))+" D "+itoa(int(s.camera.Distance+0.5)), scaleBody, Color{0.9, 0.9, 1, 1}); y += lineHeight
    case CameraModeTopDown:
        s.ui.DrawText(x, y, "CAMT H "+itoa(int(s.camera.TopDownHeight+0.5)), scaleBody, Color{0.9, 0.9, 1, 1}); y += lineHeight
    case CameraModeFPV:
        s.ui.DrawText(x, y, "CAMV", scaleBody, Color{0.9, 0.9, 1, 1}); y += lineHeight
    }

    // PID telemetry (Altitude)
    s.ui.DrawText(x, y, "PID ALT E "+fmt2(s.drone.AltitudePID.LastError)+" I "+fmt2(s.drone.AltitudePID.Integral)+" D "+fmt2(s.drone.AltitudePID.LastDerivative), scaleBody, Color{0.9, 1, 0.9, 1}); y += lineHeight
    s.ui.DrawText(x, y, "PID ALT OUT "+fmt2(s.drone.AltitudePID.LastOutput), scaleBody, Color{0.95, 1, 0.95, 1}); y += lineHeight

    s.ui.Flush()
    gl.Disable(gl.SCISSOR_TEST)
    gl.Enable(gl.DEPTH_TEST)
}

// Simple integer to string without fmt to avoid allocation overhead in UI loop
func itoa(v int) string {
    if v == 0 { return "0" }
    neg := false
    if v < 0 { neg = true; v = -v }
    var buf [16]byte
    i := len(buf)
    for v > 0 {
        i--
        buf[i] = byte('0' + (v % 10))
        v /= 10
    }
    if neg { i--; buf[i] = '-' }
    return string(buf[i:])
}

// Format with 1 decimal place without fmt.
func fmt1(x float64) string {
    neg := x < 0
    if neg { x = -x }
    ip := int(x)
    fp := int((x - float64(ip))*10.0 + 0.5)
    s := itoa(ip) + "." + itoa(fp)
    if neg { return "-" + s }
    return s
}

// Format with 2 decimal places without fmt.
func fmt2(x float64) string {
    neg := x < 0
    if neg { x = -x }
    ip := int(x)
    fp := int((x - float64(ip))*100.0 + 0.5)
    // Ensure two digits for fractional part
    frac := itoa(fp)
    if fp < 10 { frac = "0" + frac }
    s := itoa(ip) + "." + frac
    if neg { return "-" + s }
    return s
}
