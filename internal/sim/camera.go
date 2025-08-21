package sim

import (
	"math"
)

type CameraMode int

const (
	CameraModeFollow CameraMode = iota
	CameraModeTopDown
	CameraModeFPV
)

type Camera struct {
	Position      Vec3
	Target        Vec3
	Up            Vec3
	Yaw           float64
	Pitch         float64
	Distance      float64
	FollowTarget  bool
	Mode          CameraMode
	TopDownHeight float64 // Height for top-down view
}

func NewCamera() *Camera {
	c := &Camera{
		Position:      Vec3{0, 0, 0}, // Will be calculated in first Update
		Target:        Vec3{0, 0, 0},
		Up:            Vec3{0, 1, 0},
		Yaw:           45,  // Look from behind-right
		Pitch:         -20, // Look down slightly
		Distance:      5,   // Closer to drone
		FollowTarget:  true,
		Mode:          CameraModeFollow,
		TopDownHeight: 20, // 20 meters above drone for top-down view
	}
	return c
}

func (c *Camera) Update(drone *Drone) {
	if c.FollowTarget {
		c.Target = drone.Position
	}

	switch c.Mode {
	case CameraModeFollow:
		c.updateFollowPosition()
	case CameraModeTopDown:
		c.updateTopDownPosition()
	case CameraModeFPV:
		c.updateFPVPosition(drone)
	}
}

// Follow camera mode (original orbital camera)
func (c *Camera) updateFollowPosition() {
	// Convert spherical coordinates to cartesian
	yawRad := c.Yaw * math.Pi / 180.0
	pitchRad := c.Pitch * math.Pi / 180.0

	// Calculate camera position relative to target
	c.Position.X = c.Target.X + c.Distance*math.Cos(pitchRad)*math.Sin(yawRad)
	c.Position.Y = c.Target.Y + c.Distance*math.Sin(pitchRad)
	c.Position.Z = c.Target.Z + c.Distance*math.Cos(pitchRad)*math.Cos(yawRad)

	// Ensure camera is above ground
	if c.Position.Y < 0.5 {
		c.Position.Y = 0.5
	}
}

// Top-down view mode (bird's eye view)
func (c *Camera) updateTopDownPosition() {
	// Position camera directly above the drone
	c.Position.X = c.Target.X
	c.Position.Y = c.Target.Y + c.TopDownHeight
	c.Position.Z = c.Target.Z

	// Look straight down
	c.Up = Vec3{0, 0, -1} // Change up vector for top-down view
}

// First-Person View (FPV) mode
func (c *Camera) updateFPVPosition(drone *Drone) {
	// Position camera at drone location with slight offset
	offset := Vec3{0, 0.1, 0} // Slightly above drone center
	c.Position = drone.Position.Add(offset)

	// Look in the direction the drone is facing
	pitch, yaw := drone.Rotation.X, drone.Rotation.Y

	// Calculate forward direction based on drone orientation
	cosPitch, sinPitch := math.Cos(pitch), math.Sin(pitch)
	cosYaw, sinYaw := math.Cos(yaw), math.Sin(yaw)

	// Forward vector in world coordinates
	forwardX := sinYaw * cosPitch
	forwardY := -sinPitch
	forwardZ := cosYaw * cosPitch

	// Target point ahead of drone
	lookDistance := 10.0
	c.Target = c.Position.Add(Vec3{forwardX, forwardY, forwardZ}.Mul(lookDistance))

	// Up vector rotated with drone
	c.Up = Vec3{0, 1, 0} // Reset to normal up vector
}

// Set camera mode
func (c *Camera) SetMode(mode CameraMode) {
	c.Mode = mode

	// Reset up vector when switching modes
	if mode != CameraModeTopDown {
		c.Up = Vec3{0, 1, 0}
	}
}

// Adjust top-down height
func (c *Camera) AdjustTopDownHeight(delta float64) {
	c.TopDownHeight += delta
	if c.TopDownHeight < 5.0 {
		c.TopDownHeight = 5.0 // Minimum 5m above
	}
	if c.TopDownHeight > 100.0 {
		c.TopDownHeight = 100.0 // Maximum 100m above
	}
}

func (c *Camera) GetViewMatrix() Mat4 {
	return LookAtMat4(c.Position, c.Target, c.Up)
}

func (c *Camera) GetProjectionMatrix(width, height int) Mat4 {
	// Ensure minimum dimensions to avoid division by zero
	if width < 1 {
		width = 1
	}
	if height < 1 {
		height = 1
	}

	aspect := float64(width) / float64(height)
	// Use more conservative FOV and near/far planes
	return PerspectiveMat4(45.0, aspect, 0.1, 1000.0)
}
