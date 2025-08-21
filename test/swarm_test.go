package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestSwarmArmingAndLeaderSwitch(t *testing.T) {
	// Create leader + 2 followers
	d0, d1, d2 := sim.NewDrone(), sim.NewDrone(), sim.NewDrone()
	s := sim.NewSwarm([]*sim.Drone{d0, d1, d2})

	// Initially disarmed
	if d1.IsArmed || d2.IsArmed {
		t.Fatalf("followers should start disarmed")
	}

	// Arm leader; followers should mirror after update
	d0.Arm()
	for i := 0; i < 20; i++ {
		s.Update(0.01)
	}
	if !d1.IsArmed || !d2.IsArmed {
		t.Fatalf("followers should arm with leader")
	}

	// Switch control to follower 1; it becomes leader logically
	s.SetLeader(1)
	// Move new leader position; ensure update doesn't overwrite leader's control
	d1.Position = sim.Vec3{X: 10, Y: 5, Z: -3}
	prevThr := 55.0
	d1.SetThrottle(prevThr)
	s.Update(0.01)
	if d1.ThrottlePercent != prevThr {
		t.Fatalf("leader throttle should not be overridden; got %v", d1.ThrottlePercent)
	}
}

func TestSwarmSetsAltitudeHoldOnFollowers(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	s := sim.NewSwarm([]*sim.Drone{d0, d1})
	d0.Arm()
	d0.Position.Y = 12.3

	s.Update(0.02)
	if d1.FlightMode != sim.FlightModeAltitudeHold {
		t.Fatalf("expected follower in AltitudeHold, got %v", d1.FlightMode)
	}
	if d1.AltitudeHold != d0.Position.Y {
		t.Fatalf("expected follower AltitudeHold=%v, got %v", d0.Position.Y, d1.AltitudeHold)
	}
}

func TestSwarmLeashAndClamp(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	s := sim.NewSwarm([]*sim.Drone{d0, d1})
	d0.Arm()
	d1.Arm()
	// Place follower far away
	d1.Position = sim.Vec3{X: 500, Y: 500, Z: 500}
	for i := 0; i < 20; i++ {
		s.Update(0.05)
	}
	// After updates, follower should be brought near leader
	dist := d1.Position.Sub(d0.Position).Length()
	if dist > 90.0 {
		t.Fatalf("follower too far after leash: %.2f m", dist)
	}
}
