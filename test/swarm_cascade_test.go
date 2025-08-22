package sim_test

import (
    sim "drone-simulator/internal/sim"
    "math"
    "testing"
)

// TestSwarmFollowerLowOscillationDuringTranslation verifies the cascaded
// position→velocity→tilt controller keeps followers stable with low
// oscillation and bounded tilt rates while the leader translates.
func TestSwarmFollowerLowOscillationDuringTranslation(t *testing.T) {
    // Swarm setup: leader + 4 followers
    n := 5
    drones := make([]*sim.Drone, 0, n)
    for i := 0; i < n; i++ {
        d := sim.NewDrone()
        // Start a little above ground to avoid takeoff transients
        d.Position.Y = 1.5
        drones = append(drones, d)
    }
    s := sim.NewSwarm(drones)
    leader := drones[0]

    // Leader configuration: armed, holds altitude, moves forward
    leader.Arm()
    leader.SetFlightMode(sim.FlightModeAltitudeHold)
    leader.AltitudeHold = 3.0
    leader.Position.Y = 3.0

    // Simulate a straight-line translation at ~2 m/s for 12 seconds
    dt := 0.01
    duration := 12.0
    v := 2.0

    // Track follower (index 1) metrics after warm-up
    warmup := 5.0
    maxRadialErr := 0.0
    maxTiltDeg := 0.0
    maxTiltRate := 0.0 // rad/s
    const R = 3.0      // formation radius used by controller

    var prevPitch, prevRoll float64
    var prevSet bool

    steps := int(duration / dt)
    for i := 0; i < steps; i++ {
        tsec := float64(i) * dt
        // Script leader kinematics (broadcast through swarm with latency internally)
        leader.Velocity = sim.Vec3{X: v, Y: 0, Z: 0}
        leader.Position = leader.Position.Add(leader.Velocity.Mul(dt))

        // Swarm controller update (followers receive commands)
        s.Update(dt)
        // Physics update for all drones
        for _, d := range drones {
            d.Update(dt)
        }

        if tsec < warmup {
            // Initialize tilt rate tracking after warmup to avoid start transients
            prevPitch = drones[1].Rotation.X
            prevRoll = drones[1].Rotation.Z
            prevSet = true
            continue
        }

        follower := drones[1]
        // Radial distance error relative to leader should stay near R
        delta := follower.Position.Sub(leader.Position)
        radial := math.Sqrt(delta.X*delta.X + delta.Z*delta.Z)
        rerr := math.Abs(radial - R)
        if rerr > maxRadialErr {
            maxRadialErr = rerr
        }

        // Tilt magnitude and rate should remain bounded
        tiltDeg := math.Max(math.Abs(sim.RadToDeg(follower.Rotation.X)), math.Abs(sim.RadToDeg(follower.Rotation.Z)))
        if tiltDeg > maxTiltDeg {
            maxTiltDeg = tiltDeg
        }
        if prevSet {
            dp := (follower.Rotation.X - prevPitch) / dt
            dr := (follower.Rotation.Z - prevRoll) / dt
            rate := math.Max(math.Abs(dp), math.Abs(dr))
            if rate > maxTiltRate {
                maxTiltRate = rate
            }
        }
        prevPitch = follower.Rotation.X
        prevRoll = follower.Rotation.Z
        prevSet = true
    }

    // Assert reasonable bounds that reflect cascaded control with tilt slew-limit
    if maxRadialErr > 1.5 {
        t.Fatalf("excessive radial oscillation: max |r-%.1f| = %.2f m", R, maxRadialErr)
    }
    if maxTiltDeg > 20.0 {
        t.Fatalf("excessive tilt: max |tilt| = %.1f deg", maxTiltDeg)
    }
    // Slew limiter is ~120 deg/s; allow margin for physics coupling
    if maxTiltRate > (2.5) { // rad/s
        t.Fatalf("excessive tilt rate: max = %.2f rad/s", maxTiltRate)
    }
}

