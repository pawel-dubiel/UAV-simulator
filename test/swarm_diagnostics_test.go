package sim_test

import (
	sim "drone-simulator/internal/sim"
	"math"
	"testing"
)

// record captures a small snapshot of key follower variables for diagnostics.
type record struct {
	t     float64
	idx   int
	dist  float64
	altD  float64
	speed float64
	pitch float64
	roll  float64
	yaw   float64
	thr   float64
}

// simulate runs a swarm scenario and returns diagnostics if a follower misbehaves.
func simulate(t *testing.T, duration, dt float64, scenario string) ([]record, bool) {
	n := 5
	drones := make([]*sim.Drone, 0, n)
	for i := 0; i < n; i++ {
		drones = append(drones, sim.NewDrone())
	}
	s := sim.NewSwarm(drones)
	leader := drones[0]
	leader.Arm()
	leader.SetFlightMode(sim.FlightModeAltitudeHold)
	leader.SetThrottle(leader.HoverThrottlePercent())

	diag := make([]record, 0, int(duration/dt))
	bad := false

	// Scenario generators
	altCmd := func(t float64) float64 {
		switch scenario {
		case "pulses":
			// Pulse between 1m and 6m every ~2s
			if int(t*0.5)%2 == 0 {
				return 6
			} // 0-2,4-6,..
			return 1
		default:
			// climb/hold/descend/hold
			base := 1.0
			if t < 3.0 {
				return base + (5.0 * (t / 3.0))
			}
			if t < 5.0 {
				return base + 5.0
			}
			if t < 8.0 {
				return base + 5.0*(1.0-(t-5.0)/3.0)
			}
			return base
		}
	}

	steps := int(duration / dt)
	for i := 0; i < steps; i++ {
		tsec := float64(i) * dt
		leader.AltitudeHold = altCmd(tsec)
		s.Update(dt)
		for _, d := range drones {
			d.Update(dt)
		}

		lp := leader.Position
		if anyNaN(lp.X, lp.Y, lp.Z) {
			t.Fatalf("leader NaN at t=%.2f: %+v", tsec, lp)
		}
		// Check followers and record extremes
		for j := 1; j < n; j++ {
			d := drones[j]
			p := d.Position
			if anyNaN(p.X, p.Y, p.Z) || anyNaN(d.Velocity.X, d.Velocity.Y, d.Velocity.Z) {
				t.Fatalf("follower %d NaN at t=%.2f pos=%+v vel=%+v", j, tsec, p, d.Velocity)
			}
			dist := p.Sub(lp).Length()
			altD := p.Y - lp.Y
			speed := d.Velocity.Length()
			if dist > 60 || math.Abs(altD) > 20 || speed > 60 {
				diag = append(diag, record{
					t: tsec, idx: j, dist: dist, altD: altD, speed: speed,
					pitch: d.Rotation.X, roll: d.Rotation.Z, yaw: d.Rotation.Y, thr: d.ThrottlePercent,
				})
				bad = true
				// Keep recording a few more samples even if bad to provide context
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

// TestSwarmAggressiveClimbStability probes follower variables over 10s under stress.
func TestSwarmAggressiveClimbStability(t *testing.T) {
	diag, bad := simulate(t, 10.0, 0.01, "pulses")
	if bad {
		// Summarize a few worst samples for visibility
		maxDist := 0.0
		worst := record{}
		for _, r := range diag {
			if r.dist > maxDist {
				maxDist = r.dist
				worst = r
			}
		}
		t.Fatalf("followers dispersed/unstable: worst t=%.2fs idx=%d dist=%.2fm altΔ=%.2fm speed=%.2fm/s pitch=%.1f° roll=%.1f° yaw=%.1f° thr=%.0f%%",
			worst.t, worst.idx, worst.dist, worst.altD, worst.speed,
			worst.pitch*180/math.Pi, worst.roll*180/math.Pi, worst.yaw*180/math.Pi, worst.thr)
	}
}
