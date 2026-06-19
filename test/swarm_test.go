package sim_test

import (
	sim "drone-simulator/internal/sim"
	"math"
	"testing"
)

func TestSwarmConfigValidation(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	cfg := swarmConfigForTest()
	cfg.NeighborRadius = 0
	if _, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commConfigForTest()); err == nil {
		t.Fatalf("expected error for invalid NeighborRadius")
	}
}

func TestSwarmStabilityAndCohesion(t *testing.T) {
	n := 8
	drones := make([]*sim.Drone, 0, n)
	for i := 0; i < n; i++ {
		d := sim.NewDrone()
		d.Position = sim.Vec3{X: float64(i%4) * 1.5, Y: 1.2, Z: float64(i/4) * 1.5}
		d.SetFlightMode(sim.FlightModeAltitudeHold)
		d.Arm()
		drones = append(drones, d)
	}

	cfg := swarmConfigForTest()
	s, err := sim.NewSwarm(drones, cfg, commConfigForTest())
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	initialSpread := s.MaxDistanceFromCentroid()
	maxSpread := initialSpread

	dt := 0.01
	steps := 1500
	for i := 0; i < steps; i++ {
		s.Update(dt)
		for _, d := range drones {
			d.Update(dt)
		}
		spread := s.MaxDistanceFromCentroid()
		if spread > maxSpread {
			maxSpread = spread
		}
	}

	if maxSpread > math.Max(8.0, initialSpread*2.0) {
		t.Fatalf("swarm spread grew too much: initial %.2f max %.2f", initialSpread, maxSpread)
	}
	if s.AvgNeighborCount() < 1.0 {
		t.Fatalf("expected at least one neighbor on average, got %.2f", s.AvgNeighborCount())
	}
}
