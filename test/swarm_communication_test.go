package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestSwarmUsesStaleCommunicationSnapshotForAltitudeTarget(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d0.Position = sim.Vec3{X: 0, Y: 2, Z: 0}
	d1.Position = sim.Vec3{X: 1, Y: 4, Z: 0}
	d0.SetFlightMode(sim.FlightModeAltitudeHold)
	d1.SetFlightMode(sim.FlightModeAltitudeHold)
	d0.Arm()
	d1.Arm()

	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Range = 100
	commCfg.Latency = 0.10
	commCfg.MaxAge = 1.0
	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.05)
	if d0.AltitudeHold != d0.Position.Y {
		t.Fatalf("expected d0 to keep own altitude before delivery, got %.2f", d0.AltitudeHold)
	}

	publishedY := d1.Position.Y
	d1.Position.Y = 9

	s.Update(0.05)
	if d0.AltitudeHold != d0.Position.Y {
		t.Fatalf("expected d0 to keep own altitude at partial latency, got %.2f", d0.AltitudeHold)
	}

	s.Update(0.05)
	if d0.AltitudeHold != publishedY {
		t.Fatalf("expected stale published altitude %.2f, got %.2f", publishedY, d0.AltitudeHold)
	}
}

func TestSwarmAvgNeighborCountUsesCommunicationSnapshots(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d0.Position = sim.Vec3{X: 0, Y: 1, Z: 0}
	d1.Position = sim.Vec3{X: 1, Y: 1, Z: 0}

	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Range = 0.5
	commCfg.Latency = 0
	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.01)
	if got := s.AvgNeighborCount(); got != 0 {
		t.Fatalf("expected no communication-visible neighbors, got %.2f", got)
	}
}
