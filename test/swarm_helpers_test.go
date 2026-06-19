package sim_test

import sim "drone-simulator/internal/sim"

func swarmConfigForTest() sim.SwarmConfig {
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
		GoalWeight:       0,
		GoalRadius:       0,
	}
}

func commConfigForTest() sim.CommConfig {
	return sim.CommConfig{
		Range:          6.0,
		Latency:        0.02,
		MaxAge:         0.5,
		LossMode:       sim.LossModeNone,
		PacketLossRate: 0,
		LossSeed:       0,
	}
}
