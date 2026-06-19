package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestSwarmUpdateRejectsNonPositiveDt(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	cfg := swarmConfigForTest()
	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commConfigForTest())
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}
	defer func() {
		if r := recover(); r == nil {
			t.Fatalf("expected panic for non-positive dt")
		}
	}()
	s.Update(0)
}
