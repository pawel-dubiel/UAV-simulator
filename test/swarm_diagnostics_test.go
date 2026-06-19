package sim_test

import (
	sim "drone-simulator/internal/sim"
	"math"
	"testing"
)

func TestSwarmNoNaNAndSpeedBounded(t *testing.T) {
	n := 10
	drones := make([]*sim.Drone, 0, n)
	for i := 0; i < n; i++ {
		d := sim.NewDrone()
		d.Position = sim.Vec3{X: float64(i) * 0.8, Y: 1.0, Z: float64(i%3) * 0.8}
		d.Velocity = sim.Vec3{X: float64(i%2) * 0.5, Y: 0, Z: float64(i%3) * -0.3}
		d.SetFlightMode(sim.FlightModeAltitudeHold)
		d.Arm()
		drones = append(drones, d)
	}

	cfg := swarmConfigForTest()
	s, err := sim.NewSwarm(drones, cfg, commConfigForTest())
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	dt := 0.01
	steps := 1200
	maxSpeed := 0.0
	for i := 0; i < steps; i++ {
		s.Update(dt)
		for _, d := range drones {
			d.Update(dt)
			if anyNaN(d.Position.X, d.Position.Y, d.Position.Z, d.Velocity.X, d.Velocity.Y, d.Velocity.Z) {
				t.Fatalf("NaN/Inf detected at step %d", i)
			}
			speed := d.Velocity.Length()
			if speed > maxSpeed {
				maxSpeed = speed
			}
		}
	}

	if maxSpeed > drones[0].MaxSpeed+2.0 {
		t.Fatalf("swarm speed exceeded envelope: max %.2f m/s", maxSpeed)
	}
}

func anyNaN(vals ...float64) bool {
	for _, v := range vals {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return true
		}
	}
	return false
}
