//go:build !test
// +build !test

package sim

import (
	"fmt"
	"log"
	"math"
	"strconv"
	"sync"
	"time"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

type Simulator struct {
	drones     []*Drone
	selected   int
	camera     *Camera
	renderer   *Renderer
	input      *InputHandler
	lastTime   time.Time
	ui         *UIRenderer
	lastDt     float64
	fps        float64
	fpsHistory []float64
	fpsIdx     int
	uiVisible  bool
	swarm      *Swarm
	audio      *AudioSystem

	mu sync.RWMutex

	// Cached UI strings to avoid per-frame allocations
	uiTopLine  string
	uiTopSel   int
	uiTopCount int
	uiTopArmed bool
	uiTopMode  FlightMode
	uiTopCam   CameraMode
}

func (s *Simulator) activeDrone() *Drone {
	if len(s.drones) == 0 {
		return nil
	}
	if s.selected < 0 {
		s.selected = 0
	}
	if s.selected >= len(s.drones) {
		s.selected = len(s.drones) - 1
	}
	return s.drones[s.selected]
}

// ActiveDrone exposes the current active drone for external callers.
func (s *Simulator) ActiveDrone() *Drone { return s.activeDrone() }

// Drones returns the underlying slice of drones for external iteration.
func (s *Simulator) Drones() []*Drone { return s.drones }

func NewSimulator() *Simulator {
	// Create a small swarm (leader + followers)
	n := 20
	drones := make([]*Drone, 0, n)
	for i := 0; i < n; i++ {
		d := NewDrone()
		d.Position = Vec3{float64(i%2) * 1.5, 0.05, float64(i/2) * 1.5}
		drones = append(drones, d)
	}
	camera := NewCamera()
	camera.Target = drones[0].Position

	s := &Simulator{
		drones:     drones,
		selected:   0,
		camera:     camera,
		renderer:   NewRenderer(),
		input:      NewInputHandler(),
		lastTime:   time.Now(),
		ui:         NewUIRenderer(),
		fpsHistory: make([]float64, 120),
		fpsIdx:     0,
		uiVisible:  true,
	}
	s.swarm = NewSwarm(s.drones)
	return s
}

// NewSimulatorHeadless constructs a simulator without GL/UI/input for CI/bench.
func NewSimulatorHeadless() *Simulator {
	// Reuse the same drone swarm initialization
	n := 20
	drones := make([]*Drone, 0, n)
	for i := 0; i < n; i++ {
		d := NewDrone()
		d.Position = Vec3{float64(i%2) * 1.5, 0.05, float64(i/2) * 1.5}
		drones = append(drones, d)
	}
	camera := NewCamera()
	camera.Target = drones[0].Position

	s := &Simulator{
		drones:     drones,
		selected:   0,
		camera:     camera,
		renderer:   nil, // headless
		input:      nil, // headless
		lastTime:   time.Now(),
		ui:         nil, // headless
		fpsHistory: make([]float64, 120),
		fpsIdx:     0,
		uiVisible:  false,
	}
	s.swarm = NewSwarm(s.drones)
	return s
}

func (s *Simulator) Run(window *glfw.Window) {
	s.input.SetupCallbacks(window)
	s.initAudio()
	if s.audio != nil {
		defer s.audio.Close()
	}
	s.camera.Update(s.activeDrone())

	fmt.Println("=== REALISTIC DRONE FLIGHT SIMULATOR ===")
	fmt.Printf("Aircraft: Consumer Quadcopter (%.0fg)\n", s.activeDrone().Mass*1000)
	fmt.Printf("Battery: %.0f%% | Status: DISARMED\n", s.activeDrone().BatteryPercent)
	fmt.Println()
	fmt.Println("SAFETY PROCEDURES:")
	fmt.Println("  1. Hold SPACE for 2 seconds to ARM")
	fmt.Println("  2. ESC - Emergency DISARM")
	fmt.Println("  3. H - Emergency hover")
	fmt.Println()
	fmt.Println("FLIGHT CONTROLS (selected drone):")
	fmt.Println("  W/S - Throttle up/down (gradual)")
	fmt.Println("  A/D - Yaw left/right")
	fmt.Println("  Q/E - Roll left/right")
	fmt.Println("  Up/Down - Pitch forward/back")
	fmt.Println("  Z - Zero throttle  X - Hover throttle  [/] - Select drone")
	fmt.Println()
	fmt.Println("FLIGHT MODES:")
	fmt.Println("  1 - Manual  2 - Altitude Hold  3 - Hover")
	fmt.Println()
	fmt.Println("CAMERA MODES:")
	fmt.Println("  C - Cycle camera modes (Follow → Top-Down → FPV)")
	fmt.Println("  FOLLOW: Right mouse drag - Orbit (yaw/pitch), Scroll - Zoom, +/- - Zoom")
	fmt.Println("          Left/Right arrows also yaw; Up/Down arrows control drone pitch")
	fmt.Println("  TOP-DOWN: Scroll or +/- - Height adjustment")
	fmt.Println("  FPV: Drone's eye view")
	fmt.Println()
	fmt.Println("FLIGHT ENVELOPE:")
	fmt.Printf("  Max Speed: %.0f m/s | Max Altitude: %.0fm\n", s.activeDrone().MaxSpeed, s.activeDrone().MaxAltitude)
	fmt.Printf("  Max Climb Rate: %.0f m/s | Battery Life: ~25min\n", s.activeDrone().MaxVerticalSpeed)
	fmt.Println()

	// Fixed timestep configuration
	target := time.Second / 120 // 120 Hz physics
	acc := time.Duration(0)
	prev := time.Now()

	telemetryTimer := 0.0

	for !window.ShouldClose() {
		now := time.Now()
		frame := now.Sub(prev)
		prev = now

		// Clamp to avoid spiral-of-death on stalls
		if frame > time.Second/4 {
			frame = time.Second / 4
		}
		acc += frame

		// For UI metrics and FPS smoothing, use frame time
		dtFrame := frame.Seconds()
		s.lastDt = dtFrame
		if dtFrame > 0 {
			currentFPS := 1.0 / dtFrame
			s.fps = s.fps*0.9 + currentFPS*0.1
		}
		s.fpsHistory[s.fpsIdx] = s.fps
		s.fpsIdx = (s.fpsIdx + 1) % len(s.fpsHistory)

		// Process input once per frame using fixed-step seconds for consistent feel
		s.processInput(target.Seconds())

		// Step simulation at a fixed rate
		steps := 0
		maxSteps := 5 // safety cap per frame
		for acc >= target && steps < maxSteps {
			s.update(target.Seconds())
			acc -= target
			steps++
		}

		if s.audio != nil {
			if err := s.audio.Update(s.camera, s.drones); err != nil {
				log.Fatal(err)
			}
		}

		// Interpolation factor for rendering between last completed updates
		alpha := float64(acc) / float64(target)
		if alpha < 0 {
			alpha = 0
		} else if alpha > 1 {
			alpha = 1
		}

		s.renderInterpolated(window, alpha)

		// Telemetry every ~2 seconds in real time
		telemetryTimer += dtFrame
		if telemetryTimer >= 2.0 {
			s.displayTelemetry()
			telemetryTimer = 0.0
		}

		window.SwapBuffers()
		glfw.PollEvents()
	}
}

// RunDecoupled runs physics in a fixed-rate goroutine and renders on the main thread.
// This reduces jitter and lets rendering fluctuate independently of simulation UPS.
func (s *Simulator) RunDecoupled(window *glfw.Window) {
    s.input.SetupCallbacks(window)
    s.initAudio()
    if s.audio != nil {
        defer s.audio.Close()
    }
    s.camera.Update(s.activeDrone())

	fmt.Println("=== REALISTIC DRONE FLIGHT SIMULATOR (decoupled) ===")
	fmt.Printf("Aircraft: Consumer Quadcopter (%.0fg)\n", s.activeDrone().Mass*1000)

	target := time.Second / 120 // 120 Hz physics
	stop := make(chan struct{})
	// Physics loop
	go func() {
		ticker := time.NewTicker(target)
		defer ticker.Stop()
		for {
			select {
			case <-ticker.C:
				s.mu.Lock()
				s.update(target.Seconds())
				s.mu.Unlock()
			case <-stop:
				return
			}
		}
	}()

	telemetryTimer := 0.0
	prev := time.Now()

	for !window.ShouldClose() {
		now := time.Now()
		frame := now.Sub(prev)
		prev = now
		dtFrame := frame.Seconds()

		// FPS smoothing and history
		s.lastDt = dtFrame
		if dtFrame > 0 {
			currentFPS := 1.0 / dtFrame
			s.fps = s.fps*0.9 + currentFPS*0.1
		}
		s.fpsHistory[s.fpsIdx] = s.fps
		s.fpsIdx = (s.fpsIdx + 1) % len(s.fpsHistory)

		// Apply input under exclusive lock to avoid races with physics
		s.mu.Lock()
		s.processInput(target.Seconds())
		s.mu.Unlock()

		if s.audio != nil {
			s.mu.RLock()
			err := s.audio.Update(s.camera, s.drones)
			s.mu.RUnlock()
			if err != nil {
				log.Fatal(err)
			}
		}

		s.renderInterpolated(window, 0)

		telemetryTimer += dtFrame
		if telemetryTimer >= 2.0 {
			s.displayTelemetry()
			telemetryTimer = 0.0
		}

		window.SwapBuffers()
		glfw.PollEvents()
	}

	close(stop)
}

func (s *Simulator) initAudio() {
	if s.audio != nil {
		return
	}
	audio, err := NewAudioSystem(len(s.drones))
	if err != nil {
		log.Fatal(err)
	}
	s.audio = audio
}

func (s *Simulator) displayTelemetry() {
	drone := s.activeDrone()
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
	if drone.Position.Y > drone.MaxAltitude*0.9 {
		fmt.Print(" | ⚠ ALTITUDE LIMIT")
	}
	if speed > drone.MaxSpeed*0.9 {
		fmt.Print(" | ⚠ MAX SPEED")
	}
}

func (s *Simulator) processInput(dt float64) {
	s.input.ProcessInput(s.activeDrone(), s.camera, dt)
	// Toggle HUD visibility
	if s.input.WasKeyPressed(glfw.KeyF1) {
		s.uiVisible = !s.uiVisible
	}
	// Cycle selected drone for control and camera focus
	if s.input.WasKeyPressed(glfw.KeyLeftBracket) {
		s.selected--
		if s.selected < 0 {
			s.selected = len(s.drones) - 1
		}
		if s.swarm != nil {
			s.swarm.SetLeader(s.selected)
		}
	}
	if s.input.WasKeyPressed(glfw.KeyRightBracket) {
		s.selected++
		if s.selected >= len(s.drones) {
			s.selected = 0
		}
		if s.swarm != nil {
			s.swarm.SetLeader(s.selected)
		}
	}
	// Re-form swarm in case of dispersion
	if s.input.WasKeyPressed(glfw.KeyR) {
		if s.swarm != nil {
			s.swarm.Reform()
		}
	}
}

func (s *Simulator) update(dt float64) {
	// Swarm control influences followers
	if s.swarm != nil {
		s.swarm.Update(dt)
	}
	// Physics update for all drones
	for _, d := range s.drones {
		d.Update(dt)
	}
	// Resolve simple inter-drone collisions (sphere-sphere)
	s.resolveDroneCollisions()
	// Camera tracks the selected drone
	s.camera.Update(s.activeDrone())
}

// resolveDroneCollisions performs a simple sphere-sphere collision resolution
// between drones to prevent interpenetration and reduce explosive overlaps.
// It adjusts positions to remove penetration and applies a normal impulse
// with small restitution. O(N^2) for small swarms.
func (s *Simulator) resolveDroneCollisions() {
	n := len(s.drones)
	if n < 2 {
		return
	}
	restitution := 0.1 // slightly bouncy
	for i := 0; i < n; i++ {
		a := s.drones[i]
		ra := droneRadius(a)
		for j := i + 1; j < n; j++ {
			b := s.drones[j]
			rb := droneRadius(b)
			// Horizontal-plane distance; allow slight vertical overlap tolerance
			delta := b.Position.Sub(a.Position)
			dist := delta.Length()
			minDist := ra + rb
			if dist <= 1e-6 {
				// Prevent division by zero; nudge apart along x
				delta = Vec3{X: minDist, Y: 0, Z: 0}
				dist = minDist
			}
			if dist < minDist {
				// Normalize normal
				nrm := delta.Mul(1.0 / dist)
				penetration := minDist - dist
				// Positional correction: move both half the penetration
				corr := nrm.Mul(0.5 * penetration)
				a.Position = a.Position.Sub(corr)
				b.Position = b.Position.Add(corr)

				// Relative velocity along normal
				relV := (b.Velocity.Sub(a.Velocity)).Dot(nrm)
				if relV < 0 { // approaching
					invMassA := 1.0 / a.Mass
					invMassB := 1.0 / b.Mass
					j := -(1.0 + restitution) * relV / (invMassA + invMassB)
					impulse := nrm.Mul(j)
					a.Velocity = a.Velocity.Sub(impulse.Mul(invMassA))
					b.Velocity = b.Velocity.Add(impulse.Mul(invMassB))
				}
				// Apply damage based on approach speed magnitude
				speed := math.Abs(relV)
				a.applyCollisionDamage(speed)
				b.applyCollisionDamage(speed)
			}
		}
	}
}

func droneRadius(d *Drone) float64 {
	// Use horizontal footprint; take max of half-length/half-width, scale slightly
	r := 0.5 * math.Max(d.Dimensions.X, d.Dimensions.Y)
	if r < 0.05 {
		r = 0.05
	}
	return r
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

	// Render all drones
	for idx, d := range s.drones {
		droneModel := d.GetTransformMatrix()
		s.renderer.SetMatrices(droneModel, view, projection)
		s.renderer.RenderDrone()
		// Optionally: could render selected highlight later
		_ = idx
	}

	// Draw UI overlay panel with telemetry on top of 3D
	if s.uiVisible {
		s.renderUI(width, height)
	}
}

// Render with interpolation factor alpha in [0,1]
func (s *Simulator) renderInterpolated(window *glfw.Window, alpha float64) {
	s.mu.RLock()
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	width, height := window.GetFramebufferSize()
	gl.Viewport(0, 0, int32(width), int32(height))

	projection := s.camera.GetProjectionMatrix(width, height)
	view := s.camera.GetViewMatrix()

	s.renderer.SetCamera(s.camera.Position)

	// Ground
	groundModel := TranslationMat4(Vec3{X: s.camera.Target.X, Y: 0, Z: s.camera.Target.Z})
	s.renderer.SetMatrices(groundModel, view, projection)
	s.renderer.RenderGround()

	// Drones (interpolated)
	for _, d := range s.drones {
		droneModel := d.GetTransformMatrixInterpolated(alpha)
		s.renderer.SetMatrices(droneModel, view, projection)
		s.renderer.RenderDrone()
	}

	if s.uiVisible {
		s.renderUI(width, height)
	}
	s.mu.RUnlock()
}

// RunHeadless executes fixed-step updates without creating a window.
// Returns the number of simulation steps performed.
func (s *Simulator) RunHeadless(steps int, ups int, dur time.Duration) int {
	if ups <= 0 {
		ups = 120
	}
	fixed := time.Second / time.Duration(ups)
	performed := 0
	start := time.Now()
	useSteps := steps > 0
	useDur := dur > 0

	for {
		if useSteps && performed >= steps {
			break
		}
		if useDur && time.Since(start) >= dur {
			break
		}
		s.update(fixed.Seconds())
		performed++
	}
	return performed
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
	s.ui.DrawText(x, y, "DRONE STATUS", scaleHeader, Color{1, 1, 1, 1})
	y += lineHeight * 2

	// Status strings handled via cached builder to avoid per-frame allocations

	// Battery status
	batt := "OK"
	if s.activeDrone().BatteryPercent <= s.activeDrone().CriticalBattery {
		batt = "CRIT"
	} else if s.activeDrone().BatteryPercent <= s.activeDrone().LowBatteryWarning {
		batt = "LOW"
	}

	// Numbers
	throttle := int(s.activeDrone().ThrottlePercent + 0.5)
	power := int(s.activeDrone().PowerDraw + 0.5)
	alt := int(s.activeDrone().Position.Y + 0.5)
	speed := int((Vec3{X: s.activeDrone().Velocity.X, Y: 0, Z: s.activeDrone().Velocity.Z}.Length()) + 0.5)
	batPct := int(s.activeDrone().BatteryPercent + 0.5)
	vspd := int(s.activeDrone().Velocity.Y + 0.5)
	fps := int(s.fps + 0.5)
	ms := int(s.lastDt*1000.0 + 0.5)

	// Lines
	s.ui.DrawText(x, y, s.topStatusText(), scaleBody, Color{0.9, 0.95, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "SIM FPS "+itoa(fps)+" DT "+itoa(ms)+"MS", scaleBody, Color{0.9, 1, 0.9, 1})
	y += lineHeight
	// FPS graph just below
	gx := x
	gy := y
	gw := panelWidth - x - 10
	gh := 48
	// background
	s.ui.AddRect(gx, gy, gw, gh, Color{0, 0, 0, 0.25})
	// draw bars right-to-left
	bars := gw
	if bars > len(s.fpsHistory) {
		bars = len(s.fpsHistory)
	}
	for i := 0; i < bars; i++ {
		idx := (s.fpsIdx - 1 - i + len(s.fpsHistory)) % len(s.fpsHistory)
		v := s.fpsHistory[idx]
		if v < 0 {
			v = 0
		}
		if v > 120 {
			v = 120
		}
		h := int((v/120.0)*float64(gh) + 0.5)
		if h < 1 {
			h = 1
		}
		s.ui.AddRect(gx+gw-1-i, gy+gh-h, 1, h, Color{0.2, 0.9, 0.4, 0.9})
	}
	y += gh + 6
	// Battery with color coding
	batColor := Color{0.8, 1, 0.8, 1}
	if s.activeDrone().BatteryPercent <= s.activeDrone().CriticalBattery {
		batColor = Color{1.0, 0.35, 0.35, 1}
	} else if s.activeDrone().BatteryPercent <= s.activeDrone().LowBatteryWarning {
		batColor = Color{1.0, 0.8, 0.3, 1}
	}
	s.ui.DrawText(x, y, "BAT "+itoa(batPct)+"%  "+batt, scaleBody, batColor)
	y += lineHeight

	// Health summary: DESTROYED / DAMAGED / OK
	healthText := "OK"
	healthColor := Color{0.8, 1.0, 0.8, 1}
	if s.activeDrone().Destroyed {
		healthText = "DESTROYED"
		healthColor = Color{1.0, 0.35, 0.35, 1}
	} else if len(s.activeDrone().Engines) > 0 {
		for _, e := range s.activeDrone().Engines {
			if !e.Functional || e.Efficiency < 0.99 {
				healthText = "DAMAGED"
				healthColor = Color{1.0, 0.8, 0.3, 1}
				break
			}
		}
	}
	s.ui.DrawText(x, y, "HEALTH "+healthText, scaleBody, healthColor)
	y += lineHeight
	// Swarm debug (if active)
	if s.swarm != nil {
		// max follower distance and comms latency
		maxd := int(s.swarm.MaxFollowerDistance() + 0.5)
		ageMs := int(s.swarm.MessageAge()*1000 + 0.5)
		s.ui.DrawText(x, y, "SWARM N "+itoa(len(s.drones))+"  FAR "+itoa(maxd)+"M  AGE "+itoa(ageMs)+"MS", scaleBody, Color{0.9, 1, 0.95, 1})
		y += lineHeight
		s.ui.DrawText(x, y, "REFORM: R  SELECT: [ ]", scaleBody, Color{0.9, 0.95, 1, 1})
		y += lineHeight
	}
	s.ui.DrawText(x, y, "THR "+itoa(throttle)+"%   PWR "+itoa(power)+"W", scaleBody, Color{1, 0.95, 0.8, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "ALT "+itoa(alt)+"   HSPD "+itoa(speed)+"   VSPD "+itoa(vspd), scaleBody, Color{1, 1, 1, 1})
	y += lineHeight

	y += lineHeight // spacer
	s.ui.DrawText(x, y, "POS", scaleBody, Color{0.7, 0.9, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "X "+itoa(int(s.activeDrone().Position.X+0.5)), scaleBody, Color{0.85, 0.9, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "Y "+itoa(int(s.activeDrone().Position.Y+0.5)), scaleBody, Color{0.85, 0.9, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "Z "+itoa(int(s.activeDrone().Position.Z+0.5)), scaleBody, Color{0.85, 0.9, 1, 1})
	y += lineHeight

	y += lineHeight // spacer
	s.ui.DrawText(x, y, "ROT", scaleBody, Color{0.7, 0.9, 1, 1})
	y += lineHeight
	deg := func(rad float64) int { return int(rad*180.0/3.14159265 + 0.5) }
	s.ui.DrawText(x, y, "P "+itoa(deg(s.activeDrone().Rotation.X)), scaleBody, Color{0.9, 0.85, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "Y "+itoa(deg(s.activeDrone().Rotation.Y)), scaleBody, Color{0.9, 0.85, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "R "+itoa(deg(s.activeDrone().Rotation.Z)), scaleBody, Color{0.9, 0.85, 1, 1})
	y += lineHeight

	// Extra telemetry
	// Air density (RHO), hover throttle approximation, power caps
	s.ui.DrawText(x, y, "RHO "+fmt2(s.activeDrone().AirDensity), scaleBody, Color{0.9, 1, 1, 1})
	y += lineHeight
	hov := int(s.activeDrone().HoverThrottlePercent() + 0.5)
	s.ui.DrawText(x, y, "TH HOV "+itoa(hov)+"%", scaleBody, Color{1, 0.95, 0.9, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "PWR MAX "+itoa(int(s.activeDrone().MaxPower+0.5))+"W  HOV "+itoa(int(s.activeDrone().HoverPower+0.5))+"W", scaleBody, Color{1, 0.95, 0.9, 1})
	y += lineHeight
	wx := int(s.activeDrone().WindVelocity.X + 0.5)
	wy := int(s.activeDrone().WindVelocity.Y + 0.5)
	wz := int(s.activeDrone().WindVelocity.Z + 0.5)
	s.ui.DrawText(x, y, "WIND X "+itoa(wx)+" Y "+itoa(wy)+" Z "+itoa(wz), scaleBody, Color{0.85, 1, 0.95, 1})
	y += lineHeight
	// Limits and targets
	s.ui.DrawText(x, y, "LIM H "+itoa(int(s.activeDrone().MaxSpeed+0.5))+"  V "+itoa(int(s.activeDrone().MaxVerticalSpeed+0.5)), scaleBody, Color{1, 0.9, 1, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "LIM ALT "+itoa(int(s.activeDrone().MaxAltitude+0.5)), scaleBody, Color{1, 0.9, 1, 1})
	y += lineHeight
	if s.activeDrone().FlightMode == FlightModeAltitudeHold || s.activeDrone().FlightMode == FlightModeHover {
		s.ui.DrawText(x, y, "ALT TGT "+itoa(int(s.activeDrone().AltitudeHold+0.5)), scaleBody, Color{0.95, 1, 0.95, 1})
		y += lineHeight
	}
	// Ground contact
	ground := "NO"
	if s.activeDrone().OnGround {
		ground = "YES"
	}
	s.ui.DrawText(x, y, "ONGROUND "+ground, scaleBody, Color{0.95, 0.95, 1, 1})
	y += lineHeight
	// Motor RPM (rounded)
	r0 := int(s.activeDrone().PropSpeeds[0] + 0.5)
	r1 := int(s.activeDrone().PropSpeeds[1] + 0.5)
	r2 := int(s.activeDrone().PropSpeeds[2] + 0.5)
	r3 := int(s.activeDrone().PropSpeeds[3] + 0.5)
	s.ui.DrawText(x, y, "PROP 0/1 "+itoa(r0)+" "+itoa(r1), scaleBody, Color{1, 0.9, 0.9, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "PROP 2/3 "+itoa(r2)+" "+itoa(r3), scaleBody, Color{1, 0.9, 0.9, 1})
	y += lineHeight
	// Motor temps
	t0 := int(s.activeDrone().MotorTempC[0] + 0.5)
	t1 := int(s.activeDrone().MotorTempC[1] + 0.5)
	t2 := int(s.activeDrone().MotorTempC[2] + 0.5)
	t3 := int(s.activeDrone().MotorTempC[3] + 0.5)
	s.ui.DrawText(x, y, "TEMP 0/1 "+itoa(t0)+"C "+itoa(t1)+"C", scaleBody, Color{1, 0.85, 0.85, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "TEMP 2/3 "+itoa(t2)+"C "+itoa(t3)+"C", scaleBody, Color{1, 0.85, 0.85, 1})
	y += lineHeight

	// Engine health (efficiency or FAIL)
	if len(s.activeDrone().Engines) > 0 {
		vals := make([]string, len(s.activeDrone().Engines))
		cols := make([]Color, len(s.activeDrone().Engines))
		for i, e := range s.activeDrone().Engines {
			if !e.Functional || e.Efficiency <= 0.01 {
				vals[i] = "FAIL"
				cols[i] = Color{1.0, 0.35, 0.35, 1}
			} else {
				pct := int(e.Efficiency*100 + 0.5)
				vals[i] = itoa(pct) + "%"
				if pct >= 90 {
					cols[i] = Color{0.8, 1.0, 0.8, 1}
				} else if pct >= 50 {
					cols[i] = Color{1.0, 0.9, 0.5, 1}
				} else {
					cols[i] = Color{1.0, 0.6, 0.4, 1}
				}
			}
		}
		// Print in pairs per line
		line := func(i, j int) {
			text := "ENG " + itoa(i) + "/" + itoa(j) + " " + vals[i] + " " + vals[j]
			// Draw the label and then overlay values in their colors
			s.ui.DrawText(x, y, text, scaleBody, Color{1, 1, 1, 1})
			// crude overlay colorization: redraw just values at approximate offsets
			// offsets tuned to monospaced 7px font with scale
			off := 0
			// compute simple offsets: after "ENG i/j " is ~10 chars; then value, space, value
			off = 10
			s.ui.DrawText(x+off*scaleBody*3, y, vals[i], scaleBody, cols[i])
			s.ui.DrawText(x+(off+len(vals[i])+1)*scaleBody*3, y, vals[j], scaleBody, cols[j])
			// advance
			y += lineHeight
		}
		if len(vals) >= 2 {
			line(0, 1)
		}
		if len(vals) >= 4 {
			line(2, 3)
		}
	}
	// Camera parameters
	switch s.camera.Mode {
	case CameraModeFollow:
		s.ui.DrawText(x, y, "CAMF Y "+itoa(int(s.camera.Yaw+0.5))+" P "+itoa(int(s.camera.Pitch+0.5))+" D "+itoa(int(s.camera.Distance+0.5)), scaleBody, Color{0.9, 0.9, 1, 1})
		y += lineHeight
	case CameraModeTopDown:
		s.ui.DrawText(x, y, "CAMT H "+itoa(int(s.camera.TopDownHeight+0.5)), scaleBody, Color{0.9, 0.9, 1, 1})
		y += lineHeight
	case CameraModeFPV:
		s.ui.DrawText(x, y, "CAMV", scaleBody, Color{0.9, 0.9, 1, 1})
		y += lineHeight
	}

	// PID telemetry (Altitude)
	s.ui.DrawText(x, y, "PID ALT E "+fmt2(s.activeDrone().AltitudePID.LastError)+" I "+fmt2(s.activeDrone().AltitudePID.Integral)+" D "+fmt2(s.activeDrone().AltitudePID.LastDerivative), scaleBody, Color{0.9, 1, 0.9, 1})
	y += lineHeight
	s.ui.DrawText(x, y, "PID ALT OUT "+fmt2(s.activeDrone().AltitudePID.LastOutput), scaleBody, Color{0.95, 1, 0.95, 1})
	y += lineHeight

	s.ui.Flush()
	gl.Disable(gl.SCISSOR_TEST)
	gl.Enable(gl.DEPTH_TEST)
}

// topStatusText builds and caches the HUD's top status line to avoid
// repeated allocations inside the render loop. It recomputes only when
// the selected drone, drone count, armed state, flight mode, or camera mode changes.
func (s *Simulator) topStatusText() string {
	sel := s.selected
	cnt := len(s.drones)
	d := s.activeDrone()
	armed := d != nil && d.IsArmed
	mode := FlightModeManual
	if d != nil {
		mode = d.FlightMode
	}
	cam := s.camera.Mode

	if s.uiTopLine != "" && s.uiTopSel == sel && s.uiTopCount == cnt && s.uiTopArmed == armed && s.uiTopMode == mode && s.uiTopCam == cam {
		return s.uiTopLine
	}

	statusStr := "DISARM"
	if armed {
		statusStr = "ARM"
	}

	modeStr := "MAN"
	switch mode {
	case FlightModeAltitudeHold:
		modeStr = "ALT"
	case FlightModeHover:
		modeStr = "HOV"
	}

	camStr := "FOL"
	switch cam {
	case CameraModeTopDown:
		camStr = "TOP"
	case CameraModeFPV:
		camStr = "FPV"
	}

	s.uiTopLine = "DRONE " + itoa(sel+1) + "/" + itoa(cnt) + "  " +
		"STAT " + statusStr + "   " +
		"MODE " + modeStr + "   " +
		"CAM " + camStr
	s.uiTopSel = sel
	s.uiTopCount = cnt
	s.uiTopArmed = armed
	s.uiTopMode = mode
	s.uiTopCam = cam
	return s.uiTopLine
}

// Simple integer to string without fmt to avoid allocation overhead in UI loop
func itoa(v int) string { return strconv.FormatInt(int64(v), 10) }

// Format with 1 decimal place without fmt.
func fmt1(x float64) string {
	neg := x < 0
	if neg {
		x = -x
	}
	ip := int(x)
	fp := int((x-float64(ip))*10.0 + 0.5)
	s := itoa(ip) + "." + itoa(fp)
	if neg {
		return "-" + s
	}
	return s
}

// Format with 2 decimal places without fmt.
func fmt2(x float64) string {
	neg := x < 0
	if neg {
		x = -x
	}
	ip := int(x)
	fp := int((x-float64(ip))*100.0 + 0.5)
	// Ensure two digits for fractional part
	frac := itoa(fp)
	if fp < 10 {
		frac = "0" + frac
	}
	s := itoa(ip) + "." + frac
	if neg {
		return "-" + s
	}
	return s
}
