package sim_test

import (
    sim "drone-simulator/internal/sim"
    "testing"
)

// Ensures drones don't buzz or penetrate the ground during takeoff-like motion.
func TestDistributedNoGroundBuzz(t *testing.T) {
    d0, d1 := sim.NewDrone(), sim.NewDrone()
    s := sim.NewSwarm([]*sim.Drone{d0, d1})
    // Start on ground
    d0.Position.Y = d0.Dimensions.Z / 2.0
    d1.Position.Y = d1.Dimensions.Z / 2.0
    d0.Arm(); d1.Arm()
    d0.SetThrottle(d0.HoverThrottlePercent())
    d1.SetThrottle(d1.HoverThrottlePercent())
    // Simulate 3 seconds with small altitude steps on drone 0
    dt := 0.01
    maxVz := 0.0
    for i := 0; i < int(3.0/dt); i++ {
        tsec := float64(i) * dt
        if tsec > 0.5 && tsec < 1.0 {
            d0.AltitudeHold = 1.0
        } else {
            d0.AltitudeHold = 0.2
        }
        s.Update(dt)
        d0.Update(dt)
        d1.Update(dt)
        if v := mathAbs(d1.Velocity.Y); v > maxVz {
            maxVz = v
        }
        if d1.Position.Y < d1.Dimensions.Z/2.0-1e-6 {
            t.Fatalf("drone dipped below ground: y=%.4f", d1.Position.Y)
        }
    }
    if maxVz > 5.0 { // arbitrary sanity bound
        t.Fatalf("excessive vertical speed near ground: %.2f m/s", maxVz)
    }
}

func mathAbs(x float64) float64 { if x < 0 { return -x }; return x }
