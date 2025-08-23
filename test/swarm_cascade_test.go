package sim_test

import (
    sim "drone-simulator/internal/sim"
    "math"
    "testing"
)

// In distributed mode, group should remain cohesive and tilt dynamics bounded
// during a scripted drift of one agent.
func TestDistributedCohesionAndBoundedTilt(t *testing.T) {
    n := 5
    drones := make([]*sim.Drone, 0, n)
    for i := 0; i < n; i++ {
        d := sim.NewDrone()
        d.Position.Y = 2.0
        d.Arm()
        d.SetThrottle(d.HoverThrottlePercent())
        drones = append(drones, d)
    }
    s := sim.NewSwarm(drones)

    dt := 0.01
    duration := 8.0

    // Track tilt metrics
    maxTiltDeg := 0.0
    maxTiltRate := 0.0
    var prevPitch, prevRoll float64
    var prevSet bool

    steps := int(duration / dt)
    for i := 0; i < steps; i++ {
        // Nudge one agent sideways to create a drift
        drones[0].Velocity = sim.Vec3{X: 0.8, Y: 0, Z: 0}
        drones[0].Position = drones[0].Position.Add(drones[0].Velocity.Mul(dt))

        s.Update(dt)
        for _, d := range drones { d.Update(dt) }

        // Compute swarm centroid and max distance for cohesion
        cen := sim.Vec3{}
        for _, d := range drones { cen = cen.Add(d.Position) }
        cen = cen.Mul(1.0/float64(n))
        maxDist := 0.0
        for _, d := range drones {
            dlen := d.Position.Sub(cen).Length()
            if dlen > maxDist { maxDist = dlen }
        }
        if i == steps-1 && maxDist > 45.0 {
            t.Fatalf("swarm dispersed too far from centroid: %.2f m", maxDist)
        }

        // Tilt bounds on any follower
        f := drones[1]
        tiltDeg := math.Max(math.Abs(sim.RadToDeg(f.Rotation.X)), math.Abs(sim.RadToDeg(f.Rotation.Z)))
        if tiltDeg > maxTiltDeg { maxTiltDeg = tiltDeg }
        if prevSet {
            dp := (f.Rotation.X - prevPitch) / dt
            dr := (f.Rotation.Z - prevRoll) / dt
            rate := math.Max(math.Abs(dp), math.Abs(dr))
            if rate > maxTiltRate { maxTiltRate = rate }
        }
        prevPitch = f.Rotation.X
        prevRoll = f.Rotation.Z
        prevSet = true
    }

    if maxTiltDeg > 25.0 {
        t.Fatalf("excessive tilt: max |tilt| = %.1f deg", maxTiltDeg)
    }
    if maxTiltRate > 2.5 {
        t.Fatalf("excessive tilt rate: max = %.2f rad/s", maxTiltRate)
    }
}
