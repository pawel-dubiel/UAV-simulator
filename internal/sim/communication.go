package sim

import (
	"errors"
	"fmt"
	"math/rand"
	"sort"
)

// PacketLossMode controls whether in-range communication attempts are dropped.
type PacketLossMode int

const (
	packetLossModeInvalid PacketLossMode = iota
	LossModeNone
	LossModeProbabilistic
)

// CommConfig defines explicit drone-to-drone neighbor exchange parameters.
type CommConfig struct {
	Range          float64
	Latency        float64
	MaxAge         float64
	LossMode       PacketLossMode
	PacketLossRate float64
	LossSeed       uint64
}

// NeighborSnapshot is an immutable sender state as seen by a receiver.
type NeighborSnapshot struct {
	SenderID    int
	Position    Vec3
	Velocity    Vec3
	IsArmed     bool
	FlightMode  FlightMode
	PublishTime float64
}

// CommDiagnostics exposes deterministic communication metrics.
type CommDiagnostics struct {
	AverageVisibleNeighbors float64
	OldestSnapshotAge       float64
	PendingDeliveries       int
	DeliveryAttempts        int
	DroppedPackets          int
}

// CommSystem owns pending deliveries and per-receiver visible neighbor state.
type CommSystem struct {
	cfg       CommConfig
	now       float64
	rng       *rand.Rand
	count     int
	pending   []scheduledDelivery
	neighbors []map[int]NeighborSnapshot

	deliveryAttempts int
	droppedPackets   int
}

type scheduledDelivery struct {
	deliverAt float64
	receiver  int
	snapshot  NeighborSnapshot
}

// NewCommSystem constructs an explicit deterministic communication system.
func NewCommSystem(drones []*Drone, cfg CommConfig) (*CommSystem, error) {
	if err := validateCommConfig(cfg); err != nil {
		return nil, err
	}
	if len(drones) == 0 {
		return nil, errors.New("comm init: need at least 1 drone")
	}
	for i, d := range drones {
		if d == nil {
			return nil, fmt.Errorf("comm init: drone %d is nil", i)
		}
	}

	c := &CommSystem{
		cfg:       cfg,
		count:     len(drones),
		neighbors: make([]map[int]NeighborSnapshot, len(drones)),
	}
	for i := range c.neighbors {
		c.neighbors[i] = make(map[int]NeighborSnapshot)
	}
	if cfg.LossMode == LossModeProbabilistic {
		c.rng = rand.New(rand.NewSource(int64(cfg.LossSeed)))
	}
	return c, nil
}

func validateCommConfig(cfg CommConfig) error {
	if cfg.Range <= 0 {
		return errors.New("comm init: Range must be > 0")
	}
	if cfg.Latency < 0 {
		return errors.New("comm init: Latency must be >= 0")
	}
	if cfg.MaxAge <= 0 {
		return errors.New("comm init: MaxAge must be > 0")
	}
	if cfg.MaxAge < cfg.Latency {
		return errors.New("comm init: MaxAge must be >= Latency")
	}
	switch cfg.LossMode {
	case LossModeNone:
		if cfg.PacketLossRate != 0 {
			return errors.New("comm init: PacketLossRate must be 0 when LossMode is None")
		}
		if cfg.LossSeed != 0 {
			return errors.New("comm init: LossSeed must be 0 when LossMode is None")
		}
	case LossModeProbabilistic:
		if cfg.PacketLossRate < 0 || cfg.PacketLossRate > 1 {
			return errors.New("comm init: PacketLossRate must be between 0 and 1")
		}
		if cfg.LossSeed == 0 {
			return errors.New("comm init: LossSeed must be nonzero when LossMode is Probabilistic")
		}
	default:
		return errors.New("comm init: LossMode must be explicit")
	}
	return nil
}

// Update publishes current drone state, schedules surviving deliveries, and
// delivers snapshots whose latency has elapsed.
func (c *CommSystem) Update(dt float64, drones []*Drone) {
	if c == nil {
		panic("comm update: system is nil")
	}
	if dt <= 0 {
		panic("comm update: dt must be > 0")
	}
	if len(drones) != c.count {
		panic(fmt.Sprintf("comm update: expected %d drones, got %d", c.count, len(drones)))
	}
	for i, d := range drones {
		if d == nil {
			panic(fmt.Sprintf("comm update: drone %d is nil", i))
		}
	}

	c.now += dt
	c.expireSnapshots()
	c.publish(drones)
	c.deliverDue()
}

// NeighborsFor returns a stable copy of visible snapshots for one receiver.
func (c *CommSystem) NeighborsFor(receiverID int) []NeighborSnapshot {
	if c == nil {
		panic("comm neighbors: system is nil")
	}
	if receiverID < 0 || receiverID >= c.count {
		panic(fmt.Sprintf("comm neighbors: receiver %d out of range", receiverID))
	}
	snapshots := c.neighbors[receiverID]
	ids := make([]int, 0, len(snapshots))
	for id := range snapshots {
		ids = append(ids, id)
	}
	sort.Ints(ids)

	out := make([]NeighborSnapshot, 0, len(ids))
	for _, id := range ids {
		out = append(out, snapshots[id])
	}
	return out
}

// Diagnostics returns deterministic aggregate communication metrics.
func (c *CommSystem) Diagnostics() CommDiagnostics {
	if c == nil {
		panic("comm diagnostics: system is nil")
	}
	totalVisible := 0
	oldest := 0.0
	for _, table := range c.neighbors {
		totalVisible += len(table)
		for _, snap := range table {
			age := c.now - snap.PublishTime
			if age > oldest {
				oldest = age
			}
		}
	}
	avg := 0.0
	if c.count > 0 {
		avg = float64(totalVisible) / float64(c.count)
	}
	return CommDiagnostics{
		AverageVisibleNeighbors: avg,
		OldestSnapshotAge:       oldest,
		PendingDeliveries:       len(c.pending),
		DeliveryAttempts:        c.deliveryAttempts,
		DroppedPackets:          c.droppedPackets,
	}
}

func (c *CommSystem) publish(drones []*Drone) {
	for senderID, sender := range drones {
		snapshot := NeighborSnapshot{
			SenderID:    senderID,
			Position:    sender.Position,
			Velocity:    sender.Velocity,
			IsArmed:     sender.IsArmed,
			FlightMode:  sender.FlightMode,
			PublishTime: c.now,
		}
		for receiverID, receiver := range drones {
			if senderID == receiverID {
				continue
			}
			if sender.Position.Sub(receiver.Position).Length() > c.cfg.Range {
				continue
			}
			c.deliveryAttempts++
			if c.dropPacket() {
				c.droppedPackets++
				continue
			}
			c.pending = append(c.pending, scheduledDelivery{
				deliverAt: c.now + c.cfg.Latency,
				receiver:  receiverID,
				snapshot:  snapshot,
			})
		}
	}
}

func (c *CommSystem) dropPacket() bool {
	switch c.cfg.LossMode {
	case LossModeNone:
		return false
	case LossModeProbabilistic:
		return c.rng.Float64() < c.cfg.PacketLossRate
	default:
		panic("comm update: invalid LossMode")
	}
}

func (c *CommSystem) deliverDue() {
	remaining := c.pending[:0]
	for _, delivery := range c.pending {
		if delivery.deliverAt <= c.now {
			c.neighbors[delivery.receiver][delivery.snapshot.SenderID] = delivery.snapshot
			continue
		}
		remaining = append(remaining, delivery)
	}
	c.pending = remaining
}

func (c *CommSystem) expireSnapshots() {
	for _, table := range c.neighbors {
		for senderID, snapshot := range table {
			if c.now-snapshot.PublishTime > c.cfg.MaxAge {
				delete(table, senderID)
			}
		}
	}
}
