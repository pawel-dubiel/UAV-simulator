package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestSwarmObjectiveConfigValidation(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()

	t.Run("goal weight", func(t *testing.T) {
		cfg := swarmConfigForTest()
		cfg.GoalWeight = -1
		if _, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commConfigForTest()); err == nil {
			t.Fatalf("expected error for negative GoalWeight")
		}
	})

	t.Run("goal radius", func(t *testing.T) {
		cfg := swarmConfigForTest()
		cfg.GoalRadius = -1
		if _, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commConfigForTest()); err == nil {
			t.Fatalf("expected error for negative GoalRadius")
		}
	})
}

func TestSwarmObjectiveDisabledHasNoDirectSeekEffect(t *testing.T) {
	selected, follower := objectiveDrones()
	cfg := swarmConfigForTest()
	cfg.NeighborRadius = 2
	cfg.GoalWeight = 0
	cfg.GoalRadius = 0
	commCfg := commConfigForTest()
	commCfg.Range = 100
	commCfg.Latency = 0

	s, err := sim.NewSwarm([]*sim.Drone{selected, follower}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}
	s.SetObjectiveDrone(0)

	s.Update(0.02)
	if follower.AngularVel.Z != 0 {
		t.Fatalf("expected no direct objective roll influence, got angular z %.6f", follower.AngularVel.Z)
	}
}

func TestSwarmObjectiveRequiresSelectedDroneSnapshot(t *testing.T) {
	selected, follower := objectiveDrones()
	cfg := swarmConfigForTest()
	cfg.NeighborRadius = 2
	cfg.GoalWeight = 3
	cfg.GoalRadius = 0
	commCfg := commConfigForTest()
	commCfg.Range = 1
	commCfg.Latency = 0

	s, err := sim.NewSwarm([]*sim.Drone{selected, follower}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}
	s.SetObjectiveDrone(0)

	s.Update(0.02)
	if follower.AngularVel.Z != 0 {
		t.Fatalf("expected no objective influence without selected snapshot, got angular z %.6f", follower.AngularVel.Z)
	}
}

func TestSwarmObjectiveInfluencesDroneWithSelectedSnapshot(t *testing.T) {
	selected, follower := objectiveDrones()
	cfg := swarmConfigForTest()
	cfg.NeighborRadius = 2
	cfg.GoalWeight = 3
	cfg.GoalRadius = 0
	commCfg := commConfigForTest()
	commCfg.Range = 100
	commCfg.Latency = 0

	s, err := sim.NewSwarm([]*sim.Drone{selected, follower}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}
	s.SetObjectiveDrone(0)

	s.Update(0.02)
	if follower.AngularVel.Z <= 0 {
		t.Fatalf("expected positive roll influence toward selected snapshot, got angular z %.6f", follower.AngularVel.Z)
	}
}

func TestSwarmCommunicationLinksReflectVisibleSnapshots(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}
	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Latency = 0

	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.01)
	links := s.CommunicationLinks()
	if len(links) != 2 {
		t.Fatalf("expected two directional links, got %d", len(links))
	}
	if links[0].ReceiverID == links[0].SenderID {
		t.Fatalf("link receiver and sender must differ")
	}
	if links[0].MaxAge != commCfg.MaxAge {
		t.Fatalf("expected max age %.2f, got %.2f", commCfg.MaxAge, links[0].MaxAge)
	}
}

func TestSwarmCommunicationLinksOmitExpiredSnapshots(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}
	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Latency = 0
	commCfg.MaxAge = 0.05

	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.01)
	d1.Position = sim.Vec3{X: 100, Y: 0, Z: 0}
	s.Update(0.06)
	if links := s.CommunicationLinks(); len(links) != 0 {
		t.Fatalf("expected expired links omitted, got %d", len(links))
	}
}

func TestSwarmDiagnosticsExposeCommunicationMetrics(t *testing.T) {
	d0, d1 := sim.NewDrone(), sim.NewDrone()
	d1.Position = sim.Vec3{X: 1, Y: 0, Z: 0}
	cfg := swarmConfigForTest()
	commCfg := commConfigForTest()
	commCfg.Latency = 0

	s, err := sim.NewSwarm([]*sim.Drone{d0, d1}, cfg, commCfg)
	if err != nil {
		t.Fatalf("unexpected swarm init error: %v", err)
	}

	s.Update(0.01)
	diag := s.Diagnostics()
	if diag.AverageCommunicationNeighbors != 1 {
		t.Fatalf("expected average visible neighbors 1, got %.2f", diag.AverageCommunicationNeighbors)
	}
	if diag.DeliveryAttempts != 2 {
		t.Fatalf("expected two delivery attempts, got %d", diag.DeliveryAttempts)
	}
	if diag.PendingDeliveries != 0 {
		t.Fatalf("expected no pending deliveries, got %d", diag.PendingDeliveries)
	}
}

func TestBuildCommunicationLineVertices(t *testing.T) {
	links := []sim.CommunicationLink{
		{
			ReceiverPosition:       sim.Vec3{X: 0, Y: 1, Z: 0},
			SenderSnapshotPosition: sim.Vec3{X: 2, Y: 1, Z: 0},
			SnapshotAge:            0.25,
			MaxAge:                 1.0,
		},
	}

	verts := sim.BuildCommunicationLineVertices(links)
	if len(verts) != 2 {
		t.Fatalf("expected two vertices for one link, got %d", len(verts))
	}
	if verts[0].Position != links[0].ReceiverPosition {
		t.Fatalf("expected first vertex at receiver position")
	}
	if verts[1].Position != links[0].SenderSnapshotPosition {
		t.Fatalf("expected second vertex at sender snapshot position")
	}
	if verts[0].Color.G <= verts[0].Color.R {
		t.Fatalf("expected fresh-ish link to remain green-dominant, got %+v", verts[0].Color)
	}
}

func objectiveDrones() (*sim.Drone, *sim.Drone) {
	selected, follower := sim.NewDrone(), sim.NewDrone()
	selected.Position = sim.Vec3{X: 10, Y: 1, Z: 0}
	follower.Position = sim.Vec3{X: 0, Y: 1, Z: 0}
	selected.Arm()
	follower.Arm()
	selected.OnGround = false
	follower.OnGround = false
	return selected, follower
}
