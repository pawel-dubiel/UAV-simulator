package main

import (
	"flag"
	"fmt"
	"time"

	sim "drone-simulator/internal/sim"
)

func main() {
	steps := flag.Int("steps", 1000, "Number of fixed updates to run")
	ups := flag.Int("ups", 240, "Fixed updates per second")
	duration := flag.Duration("duration", 0, "Duration to run if steps=0 (e.g., 2s)")
	arm := flag.Bool("arm", true, "Auto-arm drones")
	flag.Parse()

	// Initialize a small swarm similar to the main simulator
	n := 20
	drones := make([]*sim.Drone, 0, n)
	for i := 0; i < n; i++ {
		d := sim.NewDrone()
		d.Position = sim.Vec3{X: float64(i%2) * 1.5, Y: 0.05, Z: float64(i/2) * 1.5}
		drones = append(drones, d)
	}
	swarm, err := sim.NewSwarm(drones, swarmConfig(), commConfig())
	if err != nil {
		panic(err)
	}
	swarm.SetObjectiveDrone(0)

	if *arm {
		for _, d := range drones {
			d.Arm()
			d.SetThrottle(d.HoverThrottlePercent())
		}
	}

	// Run fixed-step updates
	var performed int
	if *steps > 0 {
		dt := 1.0 / float64(max(1, *ups))
		for i := 0; i < *steps; i++ {
			swarm.Update(dt)
			for _, d := range drones {
				d.Update(dt)
			}
			performed++
		}
	} else {
		// Run for a duration
		if *duration <= 0 {
			d := time.Second
			duration = &d
		}
		target := time.Duration(float64(time.Second) / float64(max(1, *ups)))
		ticker := time.NewTicker(target)
		defer ticker.Stop()
		deadline := time.Now().Add(*duration)
		for time.Now().Before(deadline) {
			<-ticker.C
			dt := target.Seconds()
			swarm.Update(dt)
			for _, d := range drones {
				d.Update(dt)
			}
			performed++
		}
	}

	// Print simple telemetry of the leader
	leader := drones[0]
	fmt.Printf("Completed %d steps. Leader pos=(%.2f, %.2f, %.2f) battery=%.1f%% throttle=%.0f%%\n",
		performed, leader.Position.X, leader.Position.Y, leader.Position.Z, leader.BatteryPercent, leader.ThrottlePercent)
}

func swarmConfig() sim.SwarmConfig {
	return sim.SwarmConfig{
		NeighborRadius:   6.0,
		SeparationRadius: 1.5,
		CohesionWeight:   0.8,
		AlignmentWeight:  0.6,
		SeparationWeight: 1.2,
		VelocityDamping:  0.4,
		MaxAccel:         3.0,
		MaxTiltDeg:       12.0,
		MaxTiltRateDeg:   120.0,
		AttitudeKp:       4.0,
		AttitudeKd:       3.0,
		MaxTorque:        0.35,
		GoalWeight:       1.5,
		GoalRadius:       2.0,
	}
}

func commConfig() sim.CommConfig {
	return sim.CommConfig{
		Range:          6.0,
		Latency:        0.02,
		MaxAge:         0.5,
		LossMode:       sim.LossModeNone,
		PacketLossRate: 0,
		LossSeed:       0,
	}
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}
