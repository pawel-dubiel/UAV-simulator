package sim_test

import (
    sim "drone-simulator/internal/sim"
    "math"
    "testing"
)

// record captures a small snapshot of key variables for diagnostics.
type record struct {
    t     float64
    idx   int
    speed float64
    pitch float64
    roll  float64
    yaw   float64
    thr   float64
}

// simulateDistributed runs a distributed scenario and flags instability.
func simulateDistributed(t *testing.T, duration, dt float64) ([]record, bool) {
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

    diag := make([]record, 0, int(duration/dt))
    bad := false

    steps := int(duration / dt)
    for i := 0; i < steps; i++ {
        tsec := float64(i) * dt
        // Script a mild sweep on one drone
        drones[0].Velocity = sim.Vec3{X: 1.0, Y: 0, Z: 0.5}
        drones[0].Position = drones[0].Position.Add(drones[0].Velocity.Mul(dt))

        s.Update(dt)
        for _, d := range drones { d.Update(dt) }

        // Check drones and record extremes
        for j := 0; j < n; j++ {
            d := drones[j]
            p := d.Position
            if anyNaN(p.X, p.Y, p.Z) || anyNaN(d.Velocity.X, d.Velocity.Y, d.Velocity.Z) {
                t.Fatalf("drone %d NaN at t=%.2f pos=%+v vel=%+v", j, tsec, p, d.Velocity)
            }
            speed := d.Velocity.Length()
            if speed > 60 {
                diag = append(diag, record{t: tsec, idx: j, speed: speed, pitch: d.Rotation.X, roll: d.Rotation.Z, yaw: d.Rotation.Y, thr: d.ThrottlePercent})
                bad = true
            }
        }
    }
    return diag, bad
}

func anyNaN(vals ...float64) bool {
    for _, v := range vals {
        if math.IsNaN(v) || math.IsInf(v, 0) {
            return true
        }
    }
    return false
}

// TestDistributedStability probes group stability over 10s under mild stress.
func TestDistributedStability(t *testing.T) {
    diag, bad := simulateDistributed(t, 10.0, 0.01)
    if bad {
        worst := record{}
        maxV := 0.0
        for _, r := range diag {
            if r.speed > maxV { maxV = r.speed; worst = r }
        }
        t.Fatalf("distributed instability: worst t=%.2fs idx=%d speed=%.2fm/s pitch=%.1f° roll=%.1f° yaw=%.1f° thr=%.0f%%",
            worst.t, worst.idx, worst.speed, worst.pitch*180/math.Pi, worst.roll*180/math.Pi, worst.yaw*180/math.Pi, worst.thr)
    }
}
