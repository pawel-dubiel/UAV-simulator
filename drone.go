package main

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
	Position    Vec3
	Velocity    Vec3
    Rotation    Vec3  // Pitch (X), Yaw (Y), Roll (Z) in radians
	AngularVel  Vec3
	
	// Aircraft specifications (based on DJI Mini 2)
	Mass           float64   // 249g for consumer drone
	MaxTakeoffMass float64   // Including payload
	Dimensions     Vec3      // L x W x H in meters
	
	// Power system
	ThrottlePercent float64  // 0-100% throttle input
	BatteryPercent  float64  // 0-100% battery remaining
	PowerDraw       float64  // Current power consumption (watts)
	MaxPower        float64  // Maximum motor power (watts)
	HoverPower      float64  // Power needed to hover (watts)
	
	// Flight envelope
	MaxSpeed        float64  // m/s horizontal speed
	MaxVerticalSpeed float64 // m/s climb/descent rate
	MaxAltitude     float64  // Service ceiling (meters)
	ServiceCeiling  float64  // Absolute ceiling (meters)
	
	// Environmental
	WindVelocity    Vec3     // Current wind vector
	AirDensity      float64  // kg/m³ (varies with altitude)
	
	// Flight systems
	FlightMode      FlightMode
	AltitudeHold    float64  // Target altitude for altitude hold mode
	IsArmed         bool     // Safety - motors armed/disarmed
	OnGround        bool     // Ground contact detection
	
	// Safety limits
	LowBatteryWarning float64 // Battery % for warning
	CriticalBattery   float64 // Battery % for forced landing
	
	// Internal state
	PropSpeeds      [4]float64 // Individual motor speeds (RPM)
	MotorTempC      [4]float64 // Motor temperatures
	
	// PID controllers for stability
	PitchPID        PIDController
	RollPID         PIDController
	YawPID          PIDController
	AltitudePID     PIDController
}

type PIDController struct {
	Kp, Ki, Kd    float64
	Integral      float64
	LastError     float64
	OutputLimit   float64
}

func NewDrone() *Drone {
	d := &Drone{
		// Initial state
		Position:   Vec3{0, 0.05, 0},  // On ground initially
		Velocity:   Vec3{0, 0, 0},
		Rotation:   Vec3{0, 0, 0},
		AngularVel: Vec3{0, 0, 0},
		
		// Physical specs (DJI Mini 2 equivalent)
		Mass:           0.249,  // 249g in kg
		MaxTakeoffMass: 0.249,  // No payload for consumer drone
		Dimensions:     Vec3{0.159, 0.202, 0.055}, // L x W x H meters
		
		// Power system (realistic values)
		ThrottlePercent: 0.0,
		BatteryPercent:  100.0,  // Start with full battery
		PowerDraw:      0.0,
		MaxPower:       100.0,   // ~100W total for 4 motors
		HoverPower:     45.0,    // ~45W needed to hover
		
		// Flight envelope (based on real specs)
		MaxSpeed:        16.0,   // 16 m/s (57.6 km/h) max horizontal speed
		MaxVerticalSpeed: 5.0,   // 5 m/s max climb/descent rate
		MaxAltitude:     500.0,  // 500m regulatory limit
		ServiceCeiling:  4000.0, // 4000m absolute ceiling
		
		// Environmental (sea level standard)
		WindVelocity: Vec3{0, 0, 0}, // No wind initially
		AirDensity:   1.225,         // kg/m³ at sea level
		
		// Flight systems
		FlightMode:   FlightModeManual,
		AltitudeHold: 0.0,
		IsArmed:      false, // Start disarmed for safety
		OnGround:     true,
		
		// Safety
		LowBatteryWarning: 30.0,  // Warning at 30%
		CriticalBattery:   10.0,  // Force land at 10%
		
		// Motor state
		PropSpeeds: [4]float64{0, 0, 0, 0},
		MotorTempC: [4]float64{20, 20, 20, 20}, // Start at ambient temp
	}
	
	// Initialize PID controllers with realistic gains
	d.PitchPID = PIDController{Kp: 2.0, Ki: 0.1, Kd: 0.3, OutputLimit: 1.0}
	d.RollPID = PIDController{Kp: 2.0, Ki: 0.1, Kd: 0.3, OutputLimit: 1.0}
	d.YawPID = PIDController{Kp: 1.5, Ki: 0.05, Kd: 0.2, OutputLimit: 1.0}
	d.AltitudePID = PIDController{Kp: 3.0, Ki: 0.2, Kd: 0.5, OutputLimit: 100.0}
	
	return d
}

func (d *Drone) Update(dt float64) {
	// Safety check - don't update if not armed
	if !d.IsArmed {
		d.ThrottlePercent = 0
		d.PropSpeeds = [4]float64{0, 0, 0, 0}
		return
	}
	
	// Update battery and power consumption
	d.updatePowerSystem(dt)
	
	// Check ground contact
	d.updateGroundContact()
	
	// Calculate air density at altitude (simplified)
	d.updateAirDensity()
	
	// Calculate forces
	gravity := Vec3{0, -9.81 * d.Mass, 0} // F = mg
	
	// Calculate thrust from throttle percentage
	thrust := d.calculateThrust()
	
	// Calculate drag (quadratic with velocity)
	drag := d.calculateDrag()
	
	// Ground effect - increased efficiency near ground
	if d.Position.Y < 2.0 { // Within 2 meters of ground
		groundEffectFactor := 1.0 + (0.15 * (2.0 - d.Position.Y) / 2.0)
		thrust = thrust.Mul(groundEffectFactor)
	}
	
	// Apply flight envelope limits
	d.enforceFlightEnvelope()
	
    // Total force calculation (no arbitrary wind force; wind is in drag via air-relative velocity)
    totalForce := gravity.Add(thrust).Add(drag)
	
	// Apply altitude hold if enabled
	if d.FlightMode == FlightModeAltitudeHold || d.FlightMode == FlightModeHover {
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
	
	// Update angular motion with stability augmentation
	d.updateAngularMotion(dt)
	
	// Safety systems
	d.updateSafetySystems()
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

// Calculate realistic thrust from throttle percentage
func (d *Drone) calculateThrust() Vec3 {
	if d.ThrottlePercent <= 0 {
		return Vec3{0, 0, 0}
	}
	
	// Thrust curve - not linear, more realistic response
	thrustFactor := d.ThrottlePercent / 100.0
	thrustFactor = thrustFactor * thrustFactor // Quadratic response
	
	// Maximum thrust = 2.5x weight for good performance
	maxThrustForce := 2.5 * d.Mass * 9.81
	thrustMagnitude := thrustFactor * maxThrustForce
	
    // Apply thrust in drone's local up direction. The simulator uses the
    // convention: Pitch (X), Yaw (Y), Roll (Z) and rotation order Yaw -> Pitch -> Roll
    // Compute world up as R * (0,1,0) with that order.
    roll, pitch, yaw := d.Rotation.Z, d.Rotation.X, d.Rotation.Y

    cosRoll, sinRoll := math.Cos(roll), math.Sin(roll)
    cosPitch, sinPitch := math.Cos(pitch), math.Sin(pitch)
    cosYaw, sinYaw := math.Cos(yaw), math.Sin(yaw)

    // After applying yaw(Y), pitch(X), roll(Z)
    upX := -cosYaw*sinRoll + sinYaw*sinPitch*cosRoll
    upY := cosPitch * cosRoll
    upZ := sinYaw*sinRoll + cosYaw*sinPitch*cosRoll

    thrustVector := Vec3{upX, upY, upZ}.Normalize().Mul(thrustMagnitude)
	
	// Update prop speeds based on throttle
	targetRPM := thrustFactor * 8000 // Max ~8000 RPM
	for i := range d.PropSpeeds {
		d.PropSpeeds[i] += (targetRPM - d.PropSpeeds[i]) * 0.1 // Motor spin-up/down
	}
	
	return thrustVector
}

// Calculate air resistance
func (d *Drone) calculateDrag() Vec3 {
    // Use air-relative velocity (accounts for wind properly)
    airVel := d.Velocity.Sub(d.WindVelocity)
    if airVel.Length() < 0.01 {
        return Vec3{0, 0, 0}
    }

    speed := airVel.Length()
    dragCoeff := 0.1 // Coefficient of drag
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
	
	// Power based on throttle + hover power
	throttleFactor := d.ThrottlePercent / 100.0
	d.PowerDraw = d.HoverPower + (d.MaxPower - d.HoverPower) * throttleFactor
	
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
    groundLevel := d.Dimensions.Z / 2.0 // Half of drone height
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
    groundLevel := d.Dimensions.Z / 2.0
    if d.Position.Y < groundLevel {
        d.Position.Y = groundLevel
		
		// Absorb landing impact
		if d.Velocity.Y < -2.0 {
			// Hard landing - potential damage
			d.Velocity.Y = 0
		} else if d.Velocity.Y < 0 {
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
	
	// Prevent excessive rotation
	maxAngle := math.Pi / 3 // 60 degrees max tilt
	if math.Abs(d.Rotation.X) > maxAngle {
		d.Rotation.X = maxAngle * math.Copysign(1, d.Rotation.X)
	}
	if math.Abs(d.Rotation.Z) > maxAngle {
		d.Rotation.Z = maxAngle * math.Copysign(1, d.Rotation.Z)
	}
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
	
	pid.Integral += error * dt
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
    return output
}

// Apply torque for rotation control
func (d *Drone) AddTorque(torque Vec3, dt float64) {
    if !d.IsArmed {
        return
    }
    
    // Convert torque to angular acceleration
    momentOfInertia := d.Mass * 0.02 // Simplified MOI
    angularAccel := torque.Mul(1.0 / momentOfInertia)
    d.AngularVel = d.AngularVel.Add(angularAccel.Mul(dt))
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
