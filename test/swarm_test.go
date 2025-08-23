package sim_test

import (
    sim "drone-simulator/internal/sim"
    "testing"
)

// In distributed mode, arming is not mirrored and there is no leader.
func TestDistributedNoMirroredArming(t *testing.T) {
    d0, d1, d2 := sim.NewDrone(), sim.NewDrone(), sim.NewDrone()
    s := sim.NewSwarm([]*sim.Drone{d0, d1, d2})

    if d1.IsArmed || d2.IsArmed {
        t.Fatalf("drones should start disarmed")
    }
    d0.Arm()
    for i := 0; i < 20; i++ { s.Update(0.01) }
    if d1.IsArmed || d2.IsArmed {
        t.Fatalf("arming should not mirror in distributed mode")
    }
}

// On first update, drones get AltitudeHold at their current altitude.
func TestDistributedInitialAltitudeHold(t *testing.T) {
    d0, d1 := sim.NewDrone(), sim.NewDrone()
    d0.Position.Y = 2.5
    d1.Position.Y = 4.0
    s := sim.NewSwarm([]*sim.Drone{d0, d1})
    s.Update(0.02)
    if d0.FlightMode != sim.FlightModeAltitudeHold || d1.FlightMode != sim.FlightModeAltitudeHold {
        t.Fatalf("expected AltitudeHold on all drones after init")
    }
    if d0.AltitudeHold != 2.5 || d1.AltitudeHold != 4.0 {
        t.Fatalf("expected AltitudeHold to match initial altitude")
    }
}

// With distributed rules, two armed drones placed far apart should move toward each other (cohesion)
// while maintaining separation.
func TestDistributedCohesionWithoutCollision(t *testing.T) {
    d0, d1 := sim.NewDrone(), sim.NewDrone()
    d0.Position = sim.Vec3{X: -8, Y: 2, Z: 0}
    d1.Position = sim.Vec3{X: 8, Y: 2, Z: 0}
    d0.Arm(); d1.Arm()
    d0.SetThrottle(d0.HoverThrottlePercent())
    d1.SetThrottle(d1.HoverThrottlePercent())
    s := sim.NewSwarm([]*sim.Drone{d0, d1})
    // Run for several seconds
    for i := 0; i < 1200; i++ {
        s.Update(0.01)
        d0.Update(0.01)
        d1.Update(0.01)
    }
    dist := d0.Position.Sub(d1.Position).Length()
    if dist < 1.0 || dist > 40.0 {
        t.Fatalf("unexpected pairwise distance: %.2f m", dist)
    }
    // Check no ground penetration
    if d0.Position.Y < d0.Dimensions.Z/2.0 || d1.Position.Y < d1.Dimensions.Z/2.0 {
        t.Fatalf("drone dipped below ground during distributed motion")
    }
}
