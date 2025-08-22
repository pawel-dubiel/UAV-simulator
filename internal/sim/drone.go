package sim

import (
    "math"
)

type FlightMode int

const (
	FlightModeManual FlightMode = iota
	FlightModeAltitudeHold
	FlightModeHover
)

type Drone struct {
	// Physical properties
	Position   Vec3
	Velocity   Vec3
	Rotation   Vec3 // Pitch (X), Yaw (Y), Roll (Z) in radians
	AngularVel Vec3

	// Previous state for render interpolation
	PrevPosition Vec3
	PrevRotation Vec3

	// Aircraft specifications (based on DJI Mini 2)
	Mass           float64 // 249g for consumer drone
	MaxTakeoffMass float64 // Including payload
	Dimensions     Vec3    // L x W x H in meters

	// Power system
	ThrottlePercent float64 // 0-100% throttle input
	BatteryPercent  float64 // 0-100% battery remaining
	PowerDraw       float64 // Current power consumption (watts)
	MaxPower        float64 // Maximum motor power (watts)
	HoverPower      float64 // Power needed to hover (watts)

	// Flight envelope
	MaxSpeed         float64 // m/s horizontal speed
	MaxVerticalSpeed float64 // m/s climb/descent rate
	MaxAltitude      float64 // Service ceiling (meters)
	ServiceCeiling   float64 // Absolute ceiling (meters)

	// Environmental
	WindVelocity Vec3    // Current wind vector
	AirDensity   float64 // kg/m³ (varies with altitude)

	// Flight systems
	FlightMode   FlightMode
	AltitudeHold float64 // Target altitude for altitude hold mode
	IsArmed      bool    // Safety - motors armed/disarmed
	OnGround     bool    // Ground contact detection

	// Safety limits
	LowBatteryWarning float64 // Battery % for warning
	CriticalBattery   float64 // Battery % for forced landing

	// Internal state
	PropSpeeds [4]float64 // Individual motor speeds (RPM)
    MotorTempC [4]float64 // Motor temperatures

    // Engines (quad) — allows per-engine failures/derating
    Engines []Engine

    // Damage state
    Destroyed bool

	// PID controllers for stability
	PitchPID    PIDController
	RollPID     PIDController
	YawPID      PIDController
    AltitudePID PIDController

    // Rotational inertia about body axes (Ix, Iy, Iz)
    Inertia Vec3
}

type PIDController struct {
	Kp, Ki, Kd     float64
	Integral       float64
	LastError      float64
	OutputLimit    float64
	LastDerivative float64
	LastOutput     float64
	IntegralLimit  float64
}

// Engine models a single rotor/prop unit.
type Engine struct {
    Position   Vec3    // Local position (X forward, Y up, Z right)
    Spin       int     // +1 = CW, -1 = CCW (yaw torque sign)
    Efficiency float64 // 0..1 multiplier for available thrust
    Functional bool    // If false, produces no thrust
    MaxThrust  float64 // N at 100% throttle per engine
    Mass       float64 // kg mass allocated to motor/arm at this position
}

func NewDrone() *Drone {
	d := &Drone{
		// Initial state
		Position:     Vec3{0, 0.05, 0}, // On ground initially
		Velocity:     Vec3{0, 0, 0},
		Rotation:     Vec3{0, 0, 0},
		AngularVel:   Vec3{0, 0, 0},
		PrevPosition: Vec3{0, 0.05, 0},
		PrevRotation: Vec3{0, 0, 0},

		// Physical specs (DJI Mini 2 equivalent)
		Mass:           0.249,                     // 249g in kg
		MaxTakeoffMass: 0.249,                     // No payload for consumer drone
		Dimensions:     Vec3{0.159, 0.202, 0.055}, // L x W x H meters

		// Power system (realistic values)
		ThrottlePercent: 0.0,
		BatteryPercent:  100.0, // Start with full battery
		PowerDraw:       0.0,
		MaxPower:        100.0, // ~100W total for 4 motors
		HoverPower:      45.0,  // ~45W needed to hover

		// Flight envelope (based on real specs)
		MaxSpeed:         16.0,   // 16 m/s (57.6 km/h) max horizontal speed
		MaxVerticalSpeed: 5.0,    // 5 m/s max climb/descent rate
		MaxAltitude:      500.0,  // 500m regulatory limit
		ServiceCeiling:   4000.0, // 4000m absolute ceiling

		// Environmental (sea level standard)
		WindVelocity: Vec3{0, 0, 0}, // No wind initially
		AirDensity:   1.225,         // kg/m³ at sea level

		// Flight systems
		FlightMode:   FlightModeManual,
		AltitudeHold: 0.0,
		IsArmed:      false, // Start disarmed for safety
		OnGround:     true,

		// Safety
		LowBatteryWarning: 30.0, // Warning at 30%
		CriticalBattery:   10.0, // Force land at 10%

		// Motor state
		PropSpeeds: [4]float64{0, 0, 0, 0},
		MotorTempC: [4]float64{20, 20, 20, 20}, // Start at ambient temp
	}

	// Initialize PID controllers with realistic gains
	d.PitchPID = PIDController{Kp: 2.0, Ki: 0.1, Kd: 0.3, OutputLimit: 1.0, IntegralLimit: 2.0}
	d.RollPID = PIDController{Kp: 2.0, Ki: 0.1, Kd: 0.3, OutputLimit: 1.0, IntegralLimit: 2.0}
	d.YawPID = PIDController{Kp: 1.5, Ki: 0.05, Kd: 0.2, OutputLimit: 1.0, IntegralLimit: 1.0}
	// Altitude PID output is treated as extra vertical force (N). Limit to ~2x weight.
	weight := d.Mass * 9.81
	altKi := 0.15
	altKp := 1.2
	altKd := 0.3
	integralLimit := 0.0
	if altKi > 0 {
		integralLimit = (2.0 * weight) / altKi
	}
	d.AltitudePID = PIDController{Kp: altKp, Ki: altKi, Kd: altKd, OutputLimit: 2.0 * weight, IntegralLimit: integralLimit}

	// Initialize engines (quad X-layout). Approximate arm offsets.
	armX := 0.10 // meters forward/back
	armZ := 0.12 // meters left/right
	// Total max thrust ~ 2.5 * weight; distribute across 4 engines
	totalMaxThrust := 2.5 * d.Mass * 9.81
	perMax := totalMaxThrust / 4.0
	d.Engines = []Engine{
		{Position: Vec3{X: armX, Y: 0, Z: armZ}, Spin: +1, Efficiency: 1.0, Functional: true, MaxThrust: perMax},  // front-right (CW)
		{Position: Vec3{X: armX, Y: 0, Z: -armZ}, Spin: -1, Efficiency: 1.0, Functional: true, MaxThrust: perMax}, // front-left (CCW)
		{Position: Vec3{X: -armX, Y: 0, Z: armZ}, Spin: -1, Efficiency: 1.0, Functional: true, MaxThrust: perMax}, // rear-right (CCW)
		{Position: Vec3{X: -armX, Y: 0, Z: -armZ}, Spin: +1, Efficiency: 1.0, Functional: true, MaxThrust: perMax},// rear-left (CW)
	}

    return d
}

func (d *Drone) Update(dt float64) {
    // Capture previous state for interpolation before mutating
    d.PrevPosition = d.Position
    d.PrevRotation = d.Rotation
    // If disarmed, cut thrust but continue physics (free-fall under gravity)
    if !d.IsArmed {
        d.ThrottlePercent = 0
        d.PropSpeeds = [4]float64{0, 0, 0, 0}
    }

	// Update battery and power consumption
	d.updatePowerSystem(dt)

	// Check ground contact
	d.updateGroundContact()

	// Calculate air density at altitude (simplified)
	d.updateAirDensity()

	// Calculate forces
	gravity := Vec3{0, -9.81 * d.Mass, 0} // F = mg

    // Calculate thrust and engine-induced torque
    thrust, motorTorque := d.calculateThrustAndTorque()

	// Calculate drag (quadratic with velocity)
	drag := d.calculateDrag()



	// Apply flight envelope limits
	d.enforceFlightEnvelope()

	// Total force calculation (no arbitrary wind force; wind is in drag via air-relative velocity)
	totalForce := gravity.Add(thrust).Add(drag)

	// Apply altitude hold if enabled
    if d.IsArmed && (d.FlightMode == FlightModeAltitudeHold || d.FlightMode == FlightModeHover) {
        altitudeCorrection := d.calculateAltitudeCorrection(dt)
        totalForce = totalForce.Add(Vec3{0, altitudeCorrection, 0})
    }

	// Calculate acceleration
	acceleration := totalForce.Mul(1.0 / d.Mass)

	// Integrate motion
	d.Velocity = d.Velocity.Add(acceleration.Mul(dt))
	d.Position = d.Position.Add(d.Velocity.Mul(dt))

	// Ground collision with realistic landing
	d.handleGroundCollision()

	// Apply torque from asymmetric thrust
	if motorTorque.X != 0 || motorTorque.Y != 0 || motorTorque.Z != 0 {
		d.AddTorque(motorTorque, dt)
	}

	// Update angular motion with stability augmentation
	d.updateAngularMotion(dt)

	// Safety systems
	d.updateSafetySystems()

	// Numerical safety: guard against NaN/Inf creeping in
	d.Position.X = sanitizeFinite(d.Position.X)
	d.Position.Y = sanitizeFinite(d.Position.Y)
	d.Position.Z = sanitizeFinite(d.Position.Z)
	d.Velocity.X = sanitizeFinite(d.Velocity.X)
	d.Velocity.Y = sanitizeFinite(d.Velocity.Y)
	d.Velocity.Z = sanitizeFinite(d.Velocity.Z)
}

// groundClearance returns the projected half-height of the oriented bounding box
// onto world up (Y), accounting for current pitch/roll. This prevents corners
// from sinking below ground when the drone tilts.
func (d *Drone) groundClearance() float64 {
    // Local half-extents mapped to world axes: X=length, Y=height, Z=width
    ex := d.Dimensions.X * 0.5
    ey := d.Dimensions.Z * 0.5 // height mapped to local Y
    ez := d.Dimensions.Y * 0.5

    // Euler order: Yaw (Y), then Pitch (X), then Roll (Z)
    pitch := d.Rotation.X
    roll := d.Rotation.Z

    cp := math.Cos(pitch)
    sp := math.Sin(pitch)
    sr := math.Sin(roll)
    cr := math.Cos(roll)

    // World Y components of local axes after rotation (see derivation):
    rightY := cp * sr      // local X axis Y-projection
    upY := cp * cr         // local Y axis Y-projection
    fwdY := -sp            // local Z axis Y-projection

    clearance := math.Abs(rightY)*ex + math.Abs(upY)*ey + math.Abs(fwdY)*ez
    // Small padding to avoid z-fighting with ground plane
    if clearance < 0 {
        clearance = 0
    }
    return clearance
}

// Realistic throttle control (0-100%)
func (d *Drone) SetThrottle(throttlePercent float64) {
	if throttlePercent < 0 {
		throttlePercent = 0
	}
	if throttlePercent > 100 {
		throttlePercent = 100
	}
	d.ThrottlePercent = throttlePercent
}

// Arm/disarm motors (safety)
func (d *Drone) Arm() {
	if d.OnGround && d.BatteryPercent > d.CriticalBattery {
		d.IsArmed = true
	}
}

func (d *Drone) Disarm() {
	d.IsArmed = false
	d.ThrottlePercent = 0
}

// Calculate thrust and resulting body torque from all engines.
func (d *Drone) calculateThrustAndTorque() (Vec3, Vec3) {
    // Spin down props if off
    if !d.IsArmed || d.ThrottlePercent <= 0 || len(d.Engines) == 0 {
        for i := range d.PropSpeeds {
            d.PropSpeeds[i] += (0 - d.PropSpeeds[i]) * 0.2
        }
        return Vec3{}, Vec3{}
    }

    // Quadratic response curve
    tf := d.ThrottlePercent / 100.0
    tf = tf * tf

    // Thrust is always along body-up; rotate to world using R = R_y * R_x * R_z
    roll, pitch, yaw := d.Rotation.Z, d.Rotation.X, d.Rotation.Y
    rot := RotationYMat4(yaw).Mul(RotationXMat4(pitch)).Mul(RotationZMat4(roll))
    up := rot.MulDirection(Vec3{0, 1, 0})

    // Ground effect factor
    ge := 1.0
    if d.Position.Y < 2.0 {
        ge = 1.0 + (0.15 * (2.0 - d.Position.Y) / 2.0)
    }

    sumFy := 0.0
    torque := Vec3{}
    yawCoeff := 0.02

    for i := 0; i < len(d.Engines) && i < len(d.PropSpeeds); i++ {
        e := d.Engines[i]
        eff := e.Efficiency
        if !e.Functional { eff = 0 }
        Fy := eff * tf * e.MaxThrust * ge
        sumFy += Fy
        // r x F with r=(x,0,z), F=(0,Fy,0) in local axes
        torque.X += -e.Position.Z * Fy
        torque.Z += e.Position.X * Fy
        torque.Y += float64(e.Spin) * yawCoeff * Fy

        targetRPM := eff * tf * 8000
        d.PropSpeeds[i] += (targetRPM - d.PropSpeeds[i]) * 0.1
    }

    thrust := up.Mul(sumFy)
    return thrust, torque
}

// Calculate air resistance
func (d *Drone) calculateDrag() Vec3 {
	// Use air-relative velocity (accounts for wind properly)
	airVel := d.Velocity.Sub(d.WindVelocity)
	if airVel.Length() < 0.01 {
		return Vec3{0, 0, 0}
	}

	speed := airVel.Length()
	dragCoeff := 0.1                             // Coefficient of drag
	frontArea := d.Dimensions.X * d.Dimensions.Z // Cross-sectional area

	// Drag = 0.5 * ρ * v² * Cd * A
	dragMagnitude := 0.5 * d.AirDensity * speed * speed * dragCoeff * frontArea

	// Drag opposes velocity direction
	dragDirection := airVel.Normalize().Mul(-1)
	return dragDirection.Mul(dragMagnitude)
}

// Update power consumption and battery
func (d *Drone) updatePowerSystem(dt float64) {
    if !d.IsArmed {
        d.PowerDraw = 2.0 // Idle power consumption
        return
    }

    // Power based on throttle distributed across engines
    throttleFactor := d.ThrottlePercent / 100.0
    if len(d.Engines) == 0 {
        d.PowerDraw = d.HoverPower + (d.MaxPower-d.HoverPower)*throttleFactor
    } else {
        basePer := d.HoverPower / 4.0
        var p float64
        for _, e := range d.Engines {
            eff := e.Efficiency
            if !e.Functional { eff = 0 }
            p += basePer*eff + (d.MaxPower/4.0-basePer)*throttleFactor*eff
        }
        d.PowerDraw = p
    }

	// Additional power for high speeds and maneuvers
	speed := d.Velocity.Length()
	if speed > 5.0 {
		d.PowerDraw += (speed - 5.0) * 2.0 // Extra power for speed
	}

	// Battery drain (assuming 2200mAh battery at 11.1V = 24.42Wh)
	batteryCapacityWh := 24.42
	powerHours := d.PowerDraw * dt / 3600.0 // Convert seconds to hours
	batteryDrain := (powerHours / batteryCapacityWh) * 100.0
	d.BatteryPercent -= batteryDrain

	if d.BatteryPercent < 0 {
		d.BatteryPercent = 0
	}
}

// Check ground contact
func (d *Drone) updateGroundContact() {
    groundLevel := d.groundClearance()
    d.OnGround = d.Position.Y <= groundLevel && math.Abs(d.Velocity.Y) < 0.1
}

// Update air density with altitude
func (d *Drone) updateAirDensity() {
	// Standard atmosphere model (simplified)
	altitude := d.Position.Y
	d.AirDensity = 1.225 * math.Exp(-altitude/8400.0) // Exponential decrease
}

// Flight envelope enforcement
func (d *Drone) enforceFlightEnvelope() {
	// Limit horizontal speed
	horizontalVel := Vec3{d.Velocity.X, 0, d.Velocity.Z}
	if horizontalVel.Length() > d.MaxSpeed {
		scaleFactor := d.MaxSpeed / horizontalVel.Length()
		d.Velocity.X *= scaleFactor
		d.Velocity.Z *= scaleFactor
	}

	// Limit vertical speed
	if d.Velocity.Y > d.MaxVerticalSpeed {
		d.Velocity.Y = d.MaxVerticalSpeed
	}
	if d.Velocity.Y < -d.MaxVerticalSpeed {
		d.Velocity.Y = -d.MaxVerticalSpeed
	}

	// Altitude limits
	if d.Position.Y > d.MaxAltitude {
		d.Position.Y = d.MaxAltitude
		if d.Velocity.Y > 0 {
			d.Velocity.Y = 0
		}
	}
}

// Altitude hold PID controller
func (d *Drone) calculateAltitudeCorrection(dt float64) float64 {
	// Provide altitude correction both for Altitude Hold and Hover modes
	if d.FlightMode == FlightModeAltitudeHold || d.FlightMode == FlightModeHover {
		return d.updatePIDController(&d.AltitudePID, d.AltitudeHold, d.Position.Y, dt)
	}
	return 0
}

// Ground collision handling
func (d *Drone) handleGroundCollision() {
    groundLevel := d.groundClearance()
    if d.Position.Y < groundLevel {
        // Capture pre-clamp downward speed for damage assessment
        impactSpeed := 0.0
        if d.Velocity.Y < 0 {
            impactSpeed = -d.Velocity.Y
        }
        d.Position.Y = groundLevel

        // Absorb landing impact
        if impactSpeed > 2.0 {
            // Hard landing - potential damage
            d.Velocity.Y = 0
            d.applyGroundImpactDamage(impactSpeed)
        } else if impactSpeed > 0 {
            // Soft landing
            d.Velocity.Y = 0
        }

		// Friction on ground
		if d.OnGround {
			d.Velocity.X *= 0.8
			d.Velocity.Z *= 0.8
		}
	}
}

// Angular motion with stability
func (d *Drone) updateAngularMotion(dt float64) {
	// Time-based stability augmentation (exponential damping)
	stabilityRate := 5.0 // 1/s
	damping := math.Exp(-stabilityRate * dt)
	d.AngularVel = d.AngularVel.Mul(damping)
	d.Rotation = d.Rotation.Add(d.AngularVel.Mul(dt))
	// Wrap yaw to [-pi, pi] to avoid unbounded growth
	for d.Rotation.Y > math.Pi {
		d.Rotation.Y -= 2 * math.Pi
	}
	for d.Rotation.Y < -math.Pi {
		d.Rotation.Y += 2 * math.Pi
	}

	// Prevent excessive rotation
	maxAngle := math.Pi / 3 // 60 degrees max tilt
	if math.Abs(d.Rotation.X) > maxAngle {
		d.Rotation.X = maxAngle * math.Copysign(1, d.Rotation.X)
	}
	if math.Abs(d.Rotation.Z) > maxAngle {
		d.Rotation.Z = maxAngle * math.Copysign(1, d.Rotation.Z)
	}
}

// Damage application helpers
func (d *Drone) Destroy() {
    d.Destroyed = true
    d.IsArmed = false
    d.ThrottlePercent = 0
    for i := range d.Engines {
        d.Engines[i].Efficiency = 0
        d.Engines[i].Functional = false
    }
}

func (d *Drone) applyGroundImpactDamage(speed float64) {
    if d.Destroyed { return }
    switch {
    case speed >= 4.0:
        d.Destroy()
    case speed >= 2.5:
        // Fail the strongest engine
        idx := d.strongestEngineIndex()
        if idx >= 0 { d.FailEngine(idx) }
    case speed >= 1.2:
        d.degradeTopEngines(2, 0.2)
    }
}

func (d *Drone) applyCollisionDamage(speed float64) {
    if d.Destroyed { return }
    switch {
    case speed >= 6.0:
        d.Destroy()
    case speed >= 3.0:
        idx := d.strongestEngineIndex()
        if idx >= 0 { d.FailEngine(idx) }
    case speed >= 1.5:
        d.degradeTopEngines(2, 0.25)
    }
}

func (d *Drone) strongestEngineIndex() int {
    best := -1
    bestEff := -1.0
    for i, e := range d.Engines {
        if !e.Functional { continue }
        if e.Efficiency > bestEff {
            bestEff = e.Efficiency
            best = i
        }
    }
    return best
}

func (d *Drone) degradeTopEngines(count int, frac float64) {
    if count <= 0 { return }
    for c := 0; c < count; c++ {
        idx := d.strongestEngineIndex()
        if idx < 0 { return }
        eff := d.Engines[idx].Efficiency * (1.0 - frac)
        if eff < 0.05 { eff = 0 }
        d.SetEngineEfficiency(idx, eff)
    }
}

func sanitizeFinite(x float64) float64 {
	if math.IsNaN(x) || math.IsInf(x, 0) {
		return 0
	}
	return x
}

// Safety systems
func (d *Drone) updateSafetySystems() {
	// Auto-disarm on critical battery
	if d.BatteryPercent <= d.CriticalBattery {
		d.Disarm()
	}

	// Force landing on low battery
	if d.BatteryPercent <= d.LowBatteryWarning && d.FlightMode != FlightModeManual {
		d.FlightMode = FlightModeManual
		d.ThrottlePercent = 25.0 // Gentle descent
	}
}

// PID controller update
func (d *Drone) updatePIDController(pid *PIDController, setpoint, current float64, dt float64) float64 {
	error := setpoint - current

	// Integrator with anti-windup
	pid.Integral += error * dt
	if pid.IntegralLimit > 0 {
		if pid.Integral > pid.IntegralLimit {
			pid.Integral = pid.IntegralLimit
		}
		if pid.Integral < -pid.IntegralLimit {
			pid.Integral = -pid.IntegralLimit
		}
	}
	derivative := (error - pid.LastError) / dt

	output := pid.Kp*error + pid.Ki*pid.Integral + pid.Kd*derivative

	// Apply output limits
	if output > pid.OutputLimit {
		output = pid.OutputLimit
	}
	if output < -pid.OutputLimit {
		output = -pid.OutputLimit
	}

	pid.LastError = error
	pid.LastDerivative = derivative
	pid.LastOutput = output
	return output
}

// Apply torque for rotation control
func (d *Drone) AddTorque(torque Vec3, dt float64) {
    if !d.IsArmed {
        return
    }

    // Convert torque to angular acceleration
    // Use per-axis inertia for more realistic response
    Ix := d.Inertia.X
    Iy := d.Inertia.Y
    Iz := d.Inertia.Z
    ax := 0.0
    ay := 0.0
    az := 0.0
    if Ix > 0 { ax = torque.X / Ix }
    if Iy > 0 { ay = torque.Y / Iy }
    if Iz > 0 { az = torque.Z / Iz }
    d.AngularVel = d.AngularVel.Add(Vec3{ax, ay, az}.Mul(dt))
}

// RecomputeInertia recalculates the inertia tensor (diagonal approximation)
// using a central rectangular prism for the body and point masses for engines.
func (d *Drone) RecomputeInertia() {
    // Remaining mass after subtracting engine masses is assigned to the body
    mBody := d.Mass
    for _, e := range d.Engines {
        mBody -= e.Mass
    }
    if mBody < 0 { mBody = 0 }

    // Rectangular prism at origin, axes aligned with body
    L := d.Dimensions.X // length (X)
    W := d.Dimensions.Y // width  (Z-right axis extent)
    H := d.Dimensions.Z // height (Y)
    c := 1.0 / 12.0
    Ix := c * mBody * (H*H + W*W)
    Iy := c * mBody * (L*L + W*W)
    Iz := c * mBody * (L*L + H*H)

    // Engines as point masses
    for _, e := range d.Engines {
        x, y, z := e.Position.X, e.Position.Y, e.Position.Z
        m := e.Mass
        Ix += m * (y*y + z*z)
        Iy += m * (x*x + z*z)
        Iz += m * (x*x + y*y)
    }

    const minMOI = 1e-6
    if Ix < minMOI { Ix = minMOI }
    if Iy < minMOI { Iy = minMOI }
    if Iz < minMOI { Iz = minMOI }
    d.Inertia = Vec3{X: Ix, Y: Iy, Z: Iz}
}

// Engine failure/derating APIs
func (d *Drone) FailEngine(i int) {
    if i < 0 || i >= len(d.Engines) { return }
    d.Engines[i].Functional = false
    d.Engines[i].Efficiency = 0
}

func (d *Drone) RepairEngine(i int) {
    if i < 0 || i >= len(d.Engines) { return }
    d.Engines[i].Functional = true
    d.Engines[i].Efficiency = 1
}

func (d *Drone) SetEngineEfficiency(i int, eff float64) {
    if i < 0 || i >= len(d.Engines) { return }
    if eff < 0 { eff = 0 }
    if eff > 1 { eff = 1 }
    d.Engines[i].Efficiency = eff
    d.Engines[i].Functional = eff > 0
}

// Set flight mode
func (d *Drone) SetFlightMode(mode FlightMode) {
	d.FlightMode = mode
	if mode == FlightModeAltitudeHold || mode == FlightModeHover {
		d.AltitudeHold = d.Position.Y
	}
}

// Hover throttle approximation for current thrust model
func (d *Drone) HoverThrottlePercent() float64 {
	// thrust = (throttle^2) * (2.5 * m * g) => throttle = sqrt(1/2.5)
	return math.Sqrt(1.0/2.5) * 100.0
}

func (d *Drone) GetTransformMatrix() Mat4 {
	translation := TranslationMat4(d.Position)
	rotX := RotationXMat4(d.Rotation.X)
	rotY := RotationYMat4(d.Rotation.Y)
	rotZ := RotationZMat4(d.Rotation.Z)

	// Scale cube geometry to match physical dimensions (meters).
	// Base cube extents: X=1.0, Y=0.4, Z=1.0. Height in geometry is 0.4, so scale Y accordingly.
	sx := d.Dimensions.X
	sy := d.Dimensions.Z / 0.4
	sz := d.Dimensions.Y
	scale := ScaleMat4(sx, sy, sz)

	// Model = T * R_y * R_x * R_z * S
	return translation.Mul(rotY).Mul(rotX).Mul(rotZ).Mul(scale)
}

// Interpolated transform between previous and current state
func (d *Drone) GetTransformMatrixInterpolated(alpha float64) Mat4 {
	if alpha < 0 {
		alpha = 0
	}
	if alpha > 1 {
		alpha = 1
	}
	// Linear interpolation for position and Euler rotations
	p := Vec3{
		X: d.PrevPosition.X + (d.Position.X-d.PrevPosition.X)*alpha,
		Y: d.PrevPosition.Y + (d.Position.Y-d.PrevPosition.Y)*alpha,
		Z: d.PrevPosition.Z + (d.Position.Z-d.PrevPosition.Z)*alpha,
	}
	r := Vec3{
		X: d.PrevRotation.X + (d.Rotation.X-d.PrevRotation.X)*alpha,
		Y: d.PrevRotation.Y + (d.Rotation.Y-d.PrevRotation.Y)*alpha,
		Z: d.PrevRotation.Z + (d.Rotation.Z-d.PrevRotation.Z)*alpha,
	}

	translation := TranslationMat4(p)
	rotX := RotationXMat4(r.X)
	rotY := RotationYMat4(r.Y)
	rotZ := RotationZMat4(r.Z)

	sx := d.Dimensions.X
	sy := d.Dimensions.Z / 0.4
	sz := d.Dimensions.Y
	scale := ScaleMat4(sx, sy, sz)

	return translation.Mul(rotY).Mul(rotX).Mul(rotZ).Mul(scale)
}
