package sim_test

import (
	sim "drone-simulator/internal/sim"
	"reflect"
	"testing"
)

func TestCommConfigValidation(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	drones := []*sim.Drone{d0, d1}

	tests := []struct {
		name   string
		mutate func(*sim.CommConfig)
	}{
		{name: "range", mutate: func(cfg *sim.CommConfig) { cfg.Range = 0 }},
		{name: "latency", mutate: func(cfg *sim.CommConfig) { cfg.Latency = -0.01 }},
		{name: "max age", mutate: func(cfg *sim.CommConfig) { cfg.MaxAge = 0 }},
		{name: "max age before latency", mutate: func(cfg *sim.CommConfig) { cfg.MaxAge = cfg.Latency / 2 }},
		{name: "missing loss mode", mutate: func(cfg *sim.CommConfig) { cfg.LossMode = 0 }},
		{name: "none with loss rate", mutate: func(cfg *sim.CommConfig) { cfg.PacketLossRate = 0.1 }},
		{name: "none with seed", mutate: func(cfg *sim.CommConfig) { cfg.LossSeed = 99 }},
		{name: "probabilistic low rate", mutate: func(cfg *sim.CommConfig) {
			cfg.LossMode = sim.LossModeProbabilistic
			cfg.PacketLossRate = -0.1
			cfg.LossSeed = 1
		}},
		{name: "probabilistic high rate", mutate: func(cfg *sim.CommConfig) {
			cfg.LossMode = sim.LossModeProbabilistic
			cfg.PacketLossRate = 1.1
			cfg.LossSeed = 1
		}},
		{name: "probabilistic missing seed", mutate: func(cfg *sim.CommConfig) {
			cfg.LossMode = sim.LossModeProbabilistic
			cfg.PacketLossRate = 0.5
			cfg.LossSeed = 0
		}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cfg := commConfigForTest()
			tt.mutate(&cfg)
			if _, err := sim.NewCommSystem(drones, cfg); err == nil {
				t.Fatalf("expected config error")
			}
		})
	}
}

func TestCommSnapshotsRespectLatencyAndPublishTimeState(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d0.Position = sim.Vec3{X: 0, Y: 1, Z: 0}
	d1.Position = sim.Vec3{X: 2, Y: 3, Z: 4}
	d1.Velocity = sim.Vec3{X: 1, Y: 2, Z: 3}

	cfg := commConfigForTest()
	cfg.Latency = 0.10
	cfg.MaxAge = 1.0
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.05, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 0 {
		t.Fatalf("expected no neighbor before latency, got %d", len(got))
	}

	publishedPos := d1.Position
	publishedVel := d1.Velocity
	d1.Position = sim.Vec3{X: 100, Y: 100, Z: 100}
	d1.Velocity = sim.Vec3{X: -9, Y: -9, Z: -9}

	comm.Update(0.05, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 0 {
		t.Fatalf("expected no neighbor at partial latency, got %d", len(got))
	}

	comm.Update(0.05, []*sim.Drone{d0, d1})
	got := comm.NeighborsFor(0)
	if len(got) != 1 {
		t.Fatalf("expected one delivered neighbor, got %d", len(got))
	}
	if got[0].SenderID != 1 {
		t.Fatalf("expected sender 1, got %d", got[0].SenderID)
	}
	if got[0].Position != publishedPos {
		t.Fatalf("expected publish-time position %+v, got %+v", publishedPos, got[0].Position)
	}
	if got[0].Velocity != publishedVel {
		t.Fatalf("expected publish-time velocity %+v, got %+v", publishedVel, got[0].Velocity)
	}
}

func TestCommOutOfRangeSnapshotsAreNotDelivered(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d0.Position = sim.Vec3{X: 0, Y: 0, Z: 0}
	d1.Position = sim.Vec3{X: 20, Y: 0, Z: 0}

	cfg := commConfigForTest()
	cfg.Latency = 0
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.01, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 0 {
		t.Fatalf("expected no out-of-range neighbor, got %d", len(got))
	}
}

func TestCommLossModeNoneDoesNotDropInRangeSnapshots(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}

	cfg := commConfigForTest()
	cfg.Latency = 0
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.01, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 1 {
		t.Fatalf("expected one in-range neighbor, got %d", len(got))
	}
	diag := comm.Diagnostics()
	if diag.DroppedPackets != 0 {
		t.Fatalf("expected no drops, got %d", diag.DroppedPackets)
	}
}

func TestCommProbabilisticRateOneDropsAllInRangeSnapshots(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}

	cfg := commConfigForTest()
	cfg.Latency = 0
	cfg.LossMode = sim.LossModeProbabilistic
	cfg.PacketLossRate = 1.0
	cfg.LossSeed = 42
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.01, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 0 {
		t.Fatalf("expected all snapshots dropped, got %d neighbors", len(got))
	}
	diag := comm.Diagnostics()
	if diag.DeliveryAttempts == 0 {
		t.Fatalf("expected delivery attempts")
	}
	if diag.DroppedPackets != diag.DeliveryAttempts {
		t.Fatalf("expected all attempts dropped, got %d/%d", diag.DroppedPackets, diag.DeliveryAttempts)
	}
}

func TestCommProbabilisticLossIsDeterministicForSeed(t *testing.T) {
	dronesA := []*sim.Drone{sim.NewDrone(), sim.NewDrone(), sim.NewDrone()}
	dronesB := []*sim.Drone{sim.NewDrone(), sim.NewDrone(), sim.NewDrone()}
	for i := range dronesA {
		pos := sim.Vec3{X: float64(i), Y: 0, Z: 0}
		dronesA[i].Position = pos
		dronesB[i].Position = pos
	}

	cfg := commConfigForTest()
	cfg.Latency = 0
	cfg.LossMode = sim.LossModeProbabilistic
	cfg.PacketLossRate = 0.5
	cfg.LossSeed = 7

	commA, err := sim.NewCommSystem(dronesA, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}
	commB, err := sim.NewCommSystem(dronesB, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	for i := 0; i < 4; i++ {
		commA.Update(0.01, dronesA)
		commB.Update(0.01, dronesB)
	}

	for id := range dronesA {
		if !reflect.DeepEqual(commA.NeighborsFor(id), commB.NeighborsFor(id)) {
			t.Fatalf("expected matching neighbors for receiver %d with same seed", id)
		}
	}
	if commA.Diagnostics() != commB.Diagnostics() {
		t.Fatalf("expected matching diagnostics with same seed")
	}
}

func TestCommSnapshotsExpireAfterMaxAge(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}

	cfg := commConfigForTest()
	cfg.Latency = 0
	cfg.MaxAge = 0.10
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.01, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 1 {
		t.Fatalf("expected visible neighbor, got %d", len(got))
	}

	d1.Position = sim.Vec3{X: 20, Y: 0, Z: 0}
	comm.Update(0.05, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 1 {
		t.Fatalf("expected stale neighbor before max age, got %d", len(got))
	}

	comm.Update(0.06, []*sim.Drone{d0, d1})
	if got := comm.NeighborsFor(0); len(got) != 0 {
		t.Fatalf("expected expired neighbor removed, got %d", len(got))
	}
}

func TestCommNeighborsForReturnsCopy(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}

	cfg := commConfigForTest()
	cfg.Latency = 0
	comm, err := sim.NewCommSystem([]*sim.Drone{d0, d1}, cfg)
	if err != nil {
		t.Fatalf("unexpected comm init error: %v", err)
	}

	comm.Update(0.01, []*sim.Drone{d0, d1})
	got := comm.NeighborsFor(0)
	if len(got) != 1 {
		t.Fatalf("expected one neighbor, got %d", len(got))
	}
	got[0].Position.X = 999

	again := comm.NeighborsFor(0)
	if again[0].Position.X == 999 {
		t.Fatalf("NeighborsFor exposed mutable internal storage")
	}
}
