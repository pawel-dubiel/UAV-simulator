package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

// Ensures altitude hold targets use neighbor average when armed.
func TestSwarmAltitudeTargetsNeighborAverage(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d0.Position.Y = 2.0
	d1.Position.Y = 4.0
	d0.SetFlightMode(sim.FlightModeAltitudeHold)
	d1.SetFlightMode(sim.FlightModeAltitudeHold)
	d0.Arm()
	d1.Arm()

	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Latency = 0
	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.02)
	if d0.AltitudeHold != d1.Position.Y {
		t.Fatalf("expected d0 AltitudeHold %.2f, got %.2f", d1.Position.Y, d0.AltitudeHold)
	}
	if d1.AltitudeHold != d0.Position.Y {
		t.Fatalf("expected d1 AltitudeHold %.2f, got %.2f", d0.Position.Y, d1.AltitudeHold)
	}
}
