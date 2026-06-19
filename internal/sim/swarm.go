package sim

import (
	"errors"
	"fmt"
	"math"
)

// SwarmConfig defines distributed swarm tuning values.
type SwarmConfig struct {
	NeighborRadius   float64
	SeparationRadius float64
	CohesionWeight   float64
	AlignmentWeight  float64
	SeparationWeight float64
	VelocityDamping  float64
	MaxAccel         float64
	MaxTiltDeg       float64
	MaxTiltRateDeg   float64
	AttitudeKp       float64
	AttitudeKd       float64
	MaxTorque        float64
	GoalWeight       float64
	GoalRadius       float64
}

// Swarm coordinates decentralized swarm motion using local neighbor rules.
type Swarm struct {
	drones       []*Drone
	cfg          SwarmConfig
	comm         *CommSystem
	ctrl         map[int]*swarmCtrlState
	objectiveID  int
	hasObjective bool
}

// SwarmDiagnostics summarizes swarm and communication state.
type SwarmDiagnostics struct {
	Spread                        float64
	AverageCommunicationNeighbors float64
	OldestSnapshotAge             float64
	PendingDeliveries             int
	DeliveryAttempts              int
	DroppedPackets                int
}

// CommunicationLink describes one visible receiver-to-sender snapshot link.
type CommunicationLink struct {
	ReceiverID             int
	SenderID               int
	ReceiverPosition       Vec3
	SenderSnapshotPosition Vec3
	SnapshotAge            float64
	MaxAge                 float64
}

// NewSwarm constructs a distributed swarm controller with explicit config.
func NewSwarm(drones []*Drone, cfg SwarmConfig, commCfg CommConfig) (*Swarm, error) {
	if len(drones) < 2 {
		return nil, fmt.Errorf("swarm init: need at least 2 drones, got %d", len(drones))
	}
	if cfg.NeighborRadius <= 0 {
		return nil, errors.New("swarm init: NeighborRadius must be > 0")
	}
	if cfg.SeparationRadius <= 0 {
		return nil, errors.New("swarm init: SeparationRadius must be > 0")
	}
	if cfg.SeparationRadius >= cfg.NeighborRadius {
		return nil, errors.New("swarm init: SeparationRadius must be < NeighborRadius")
	}
	if cfg.CohesionWeight <= 0 {
		return nil, errors.New("swarm init: CohesionWeight must be > 0")
	}
	if cfg.AlignmentWeight < 0 {
		return nil, errors.New("swarm init: AlignmentWeight must be >= 0")
	}
	if cfg.SeparationWeight <= 0 {
		return nil, errors.New("swarm init: SeparationWeight must be > 0")
	}
	if cfg.VelocityDamping < 0 {
		return nil, errors.New("swarm init: VelocityDamping must be >= 0")
	}
	if cfg.MaxAccel <= 0 {
		return nil, errors.New("swarm init: MaxAccel must be > 0")
	}
	if cfg.MaxTiltDeg <= 0 {
		return nil, errors.New("swarm init: MaxTiltDeg must be > 0")
	}
	if cfg.MaxTiltRateDeg <= 0 {
		return nil, errors.New("swarm init: MaxTiltRateDeg must be > 0")
	}
	if cfg.AttitudeKp <= 0 {
		return nil, errors.New("swarm init: AttitudeKp must be > 0")
	}
	if cfg.AttitudeKd <= 0 {
		return nil, errors.New("swarm init: AttitudeKd must be > 0")
	}
	if cfg.MaxTorque <= 0 {
		return nil, errors.New("swarm init: MaxTorque must be > 0")
	}
	if cfg.GoalWeight < 0 {
		return nil, errors.New("swarm init: GoalWeight must be >= 0")
	}
	if cfg.GoalRadius < 0 {
		return nil, errors.New("swarm init: GoalRadius must be >= 0")
	}
	for i, d := range drones {
		if d == nil {
			return nil, fmt.Errorf("swarm init: drone %d is nil", i)
		}
	}
	comm, err := NewCommSystem(drones, commCfg)
	if err != nil {
		return nil, fmt.Errorf("swarm init: %w", err)
	}
	return &Swarm{drones: drones, cfg: cfg, comm: comm, ctrl: make(map[int]*swarmCtrlState), objectiveID: -1}, nil
}

// SetObjectiveDrone makes drones seek the selected drone when they have its snapshot.
func (s *Swarm) SetObjectiveDrone(id int) {
	if id < 0 || id >= len(s.drones) {
		panic(fmt.Sprintf("swarm objective: drone %d out of range", id))
	}
	s.objectiveID = id
	s.hasObjective = true
}

// ClearObjective disables the selected-drone swarm objective.
func (s *Swarm) ClearObjective() {
	s.objectiveID = -1
	s.hasObjective = false
}

// Diagnostics returns aggregate swarm and communication metrics.
func (s *Swarm) Diagnostics() SwarmDiagnostics {
	comm := s.comm.Diagnostics()
	return SwarmDiagnostics{
		Spread:                        s.MaxDistanceFromCentroid(),
		AverageCommunicationNeighbors: comm.AverageVisibleNeighbors,
		OldestSnapshotAge:             comm.OldestSnapshotAge,
		PendingDeliveries:             comm.PendingDeliveries,
		DeliveryAttempts:              comm.DeliveryAttempts,
		DroppedPackets:                comm.DroppedPackets,
	}
}

// CommunicationLinks returns visible communication links for rendering.
func (s *Swarm) CommunicationLinks() []CommunicationLink {
	if s.comm == nil {
		panic("swarm links: communication system is nil")
	}
	links := make([]CommunicationLink, 0)
	for receiverID, receiver := range s.drones {
		if receiver == nil {
			panic(fmt.Sprintf("swarm links: drone %d is nil", receiverID))
		}
		for _, snapshot := range s.comm.NeighborsFor(receiverID) {
			links = append(links, CommunicationLink{
				ReceiverID:             receiverID,
				SenderID:               snapshot.SenderID,
				ReceiverPosition:       receiver.Position,
				SenderSnapshotPosition: snapshot.Position,
				SnapshotAge:            s.comm.now - snapshot.PublishTime,
				MaxAge:                 s.comm.cfg.MaxAge,
			})
		}
	}
	return links
}

// Update applies local neighbor rules (cohesion, alignment, separation).
func (s *Swarm) Update(dt float64) {
	if dt <= 0 {
		panic("swarm update: dt must be > 0")
	}
	if len(s.drones) == 0 {
		return
	}
	if s.comm == nil {
		panic("swarm update: communication system is nil")
	}
	s.comm.Update(dt, s.drones)

	g := 9.81
	maxTilt := s.cfg.MaxTiltDeg * math.Pi / 180.0
	maxTiltRate := s.cfg.MaxTiltRateDeg * math.Pi / 180.0

	for i, d := range s.drones {
		if d == nil {
			panic(fmt.Sprintf("swarm update: drone %d is nil", i))
		}
		neighbors := 0
		var sumPos Vec3
		var sumVel Vec3
		var separation Vec3

		snapshots := s.comm.NeighborsFor(i)
		for _, other := range snapshots {
			dx := other.Position.X - d.Position.X
			dz := other.Position.Z - d.Position.Z
			dist := math.Hypot(dx, dz)
			if dist <= s.cfg.NeighborRadius {
				neighbors++
				sumPos = sumPos.Add(other.Position)
				sumVel = sumVel.Add(other.Velocity)
				if dist <= s.cfg.SeparationRadius && dist > 1e-6 {
					strength := (s.cfg.SeparationRadius - dist) / s.cfg.SeparationRadius
					sep := Vec3{X: -dx / dist * strength, Y: 0, Z: -dz / dist * strength}
					separation = separation.Add(sep)
				}
			}
		}

		var acc Vec3
		if neighbors > 0 {
			inv := 1.0 / float64(neighbors)
			avgPos := sumPos.Mul(inv)
			avgVel := sumVel.Mul(inv)
			cohesion := avgPos.Sub(d.Position)
			alignment := avgVel.Sub(d.Velocity)
			cohesion.Y = 0
			alignment.Y = 0
			separation.Y = 0

			acc = cohesion.Mul(s.cfg.CohesionWeight).
				Add(alignment.Mul(s.cfg.AlignmentWeight)).
				Add(separation.Mul(s.cfg.SeparationWeight))
		}

		if goal := s.goalAcceleration(i, d, snapshots); goal.Length() > 0 {
			acc = acc.Add(goal)
		}

		acc = acc.Add(Vec3{X: -d.Velocity.X * s.cfg.VelocityDamping, Y: 0, Z: -d.Velocity.Z * s.cfg.VelocityDamping})
		if acc.Length() > s.cfg.MaxAccel {
			acc = acc.Normalize().Mul(s.cfg.MaxAccel)
		}

		pitchCmd := clamp(-math.Atan2(acc.Z, g), -maxTilt, maxTilt)
		rollCmd := clamp(math.Atan2(acc.X, g), -maxTilt, maxTilt)

		st := s.ctrl[i]
		if st == nil {
			st = &swarmCtrlState{prevPitch: d.Rotation.X, prevRoll: d.Rotation.Z}
			s.ctrl[i] = st
		}
		pitchTarget := slew(st.prevPitch, pitchCmd, maxTiltRate, dt)
		rollTarget := slew(st.prevRoll, rollCmd, maxTiltRate, dt)
		st.prevPitch = pitchTarget
		st.prevRoll = rollTarget

		torqueX := s.cfg.AttitudeKp*(pitchTarget-d.Rotation.X) - s.cfg.AttitudeKd*d.AngularVel.X
		torqueZ := s.cfg.AttitudeKp*(rollTarget-d.Rotation.Z) - s.cfg.AttitudeKd*d.AngularVel.Z

		agl := d.Position.Y - d.groundClearance()
		attScale := clamp((agl-0.05)/0.35, 0.0, 1.0)
		torqueX *= attScale
		torqueZ *= attScale

		if d.OnGround {
			torqueX = 0
			torqueZ = 0
		}

		if torqueX > s.cfg.MaxTorque {
			torqueX = s.cfg.MaxTorque
		} else if torqueX < -s.cfg.MaxTorque {
			torqueX = -s.cfg.MaxTorque
		}
		if torqueZ > s.cfg.MaxTorque {
			torqueZ = s.cfg.MaxTorque
		} else if torqueZ < -s.cfg.MaxTorque {
			torqueZ = -s.cfg.MaxTorque
		}

		d.AddTorque(Vec3{X: torqueX, Y: 0, Z: torqueZ}, dt)

		if d.IsArmed && (d.FlightMode == FlightModeAltitudeHold || d.FlightMode == FlightModeHover) {
			altTarget := d.Position.Y
			if neighbors > 0 {
				altTarget = sumPos.Y / float64(neighbors)
			}
			d.AltitudeHold = altTarget
		}
	}
}

func (s *Swarm) goalAcceleration(droneID int, d *Drone, snapshots []NeighborSnapshot) Vec3 {
	if !s.hasObjective || s.cfg.GoalWeight == 0 || droneID == s.objectiveID {
		return Vec3{}
	}
	for _, snapshot := range snapshots {
		if snapshot.SenderID != s.objectiveID {
			continue
		}
		delta := snapshot.Position.Sub(d.Position)
		delta.Y = 0
		dist := delta.Length()
		if dist <= s.cfg.GoalRadius || dist <= 1e-6 {
			return Vec3{}
		}
		return delta.Normalize().Mul(s.cfg.GoalWeight)
	}
	return Vec3{}
}

// MaxDistanceFromCentroid returns the maximum distance from swarm centroid.
func (s *Swarm) MaxDistanceFromCentroid() float64 {
	if len(s.drones) == 0 {
		return 0
	}
	centroid := Vec3{}
	for i, d := range s.drones {
		if d == nil {
			panic(fmt.Sprintf("swarm metric: drone %d is nil", i))
		}
		centroid = centroid.Add(d.Position)
	}
	centroid = centroid.Mul(1.0 / float64(len(s.drones)))
	maxd := 0.0
	for _, d := range s.drones {
		dist := d.Position.Sub(centroid).Length()
		if dist > maxd {
			maxd = dist
		}
	}
	return maxd
}

// AvgNeighborCount returns the average number of neighbors per drone.
func (s *Swarm) AvgNeighborCount() float64 {
	if len(s.drones) == 0 {
		return 0
	}
	if s.comm == nil {
		panic("swarm metric: communication system is nil")
	}
	total := 0
	for i, d := range s.drones {
		if d == nil {
			panic(fmt.Sprintf("swarm metric: drone %d is nil", i))
		}
		count := 0
		for _, other := range s.comm.NeighborsFor(i) {
			dx := other.Position.X - d.Position.X
			dz := other.Position.Z - d.Position.Z
			if math.Hypot(dx, dz) <= s.cfg.NeighborRadius {
				count++
			}
		}
		total += count
	}
	return float64(total) / float64(len(s.drones))
}

// swarmCtrlState holds per-drone control memory for smoothing.
type swarmCtrlState struct {
	prevPitch float64
	prevRoll  float64
}

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

// slew limits the rate of change from prev toward target by maxRate (units/sec)
func slew(prev, target, maxRate, dt float64) float64 {
	maxDelta := maxRate * dt
	delta := target - prev
	if delta > maxDelta {
		delta = maxDelta
	} else if delta < -maxDelta {
		delta = -maxDelta
	}
	return prev + delta
}
