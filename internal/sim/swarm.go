package sim

import (
	"math"
)

// Swarm coordinates simple follower behavior around a leader (index 0).
// Followers maintain offsets around the leader and match altitude using the drone's PID.
type Swarm struct {
	drones []*Drone
	// simple broadcast of leader state with latency
	simTime   float64
	latency   float64
	queue     []scheduledMsg
	last      LeaderState
	hasLast   bool
	leaderIdx int
}

func NewSwarm(drones []*Drone) *Swarm {
	return &Swarm{drones: drones, latency: 0.1, leaderIdx: 0}
}

func (s *Swarm) SetLeader(idx int) {
	if idx < 0 || idx >= len(s.drones) {
		return
	}
	s.leaderIdx = idx
}

func (s *Swarm) Update(dt float64) {
	if len(s.drones) == 0 {
		return
	}
	s.simTime += dt
	leader := s.drones[s.leaderIdx]

	// Broadcast leader state with latency
	msg := LeaderState{
		Position:   leader.Position,
		Velocity:   leader.Velocity,
		Yaw:        leader.Rotation.Y,
		FlightMode: leader.FlightMode,
		Throttle:   leader.ThrottlePercent,
		IsArmed:    leader.IsArmed,
	}
	s.queue = append(s.queue, scheduledMsg{deliverAt: s.simTime + s.latency, state: msg})
	// Deliver due messages
	for len(s.queue) > 0 && s.queue[0].deliverAt <= s.simTime {
		s.last = s.queue[0].state
		s.hasLast = true
		s.queue = s.queue[1:]
	}

	// Auto-arm/disarm followers to mirror leader's armed state
	for i := 0; i < len(s.drones); i++ {
		if i == s.leaderIdx {
			continue
		}
		d := s.drones[i]
		if s.last.IsArmed {
			if !d.IsArmed {
				d.Arm()
			}
		} else {
			if d.IsArmed {
				d.Disarm()
			}
		}
	}

	// Formation: circle around leader at radius R
	R := 3.0
	count := len(s.drones)
	followers := count - 1
	rank := 0
	// Controller gains (sane base)
    kpPos := 0.8  // m -> m/s^2
    kdVel := 1.2  // (m/s) -> m/s^2
    kpAtt := 4.0  // reduced attitude P to avoid flips
    kdAtt := 3.0  // rad/s damping
    kpYaw := 1.2  // gentler yaw coupling
    kdYaw := 0.8
	g := 9.81
	base := leader.Position
	lvel := leader.Velocity
	lyaw := leader.Rotation.Y
	lpitch := leader.Rotation.X
	lroll := leader.Rotation.Z
	if s.hasLast {
		base = s.last.Position
		lvel = s.last.Velocity
		lyaw = s.last.Yaw
	}
	// Allow followers to form up only after leader starts moving horizontally
	// or is clearly airborne. This prevents sliding to side before takeoff.
	movingHoriz := math.Hypot(lvel.X, lvel.Z) > 0.2
	airborne := !leader.OnGround && (leader.Position.Y > leader.Dimensions.Z/2.0+0.1)
	allowFormation := movingHoriz || airborne
	for i := 0; i < count; i++ {
		if i == s.leaderIdx {
			continue
		}
		follower := s.drones[i]
		angle := 0.0
		if followers > 0 {
			angle = 2 * math.Pi * float64(rank) / float64(followers)
		}
		offset := Vec3{R * math.Cos(angle), 0, R * math.Sin(angle)}
		targetPos := base.Add(offset)
            // Altitude follow: simple setpoint to leader altitude
            follower.SetFlightMode(FlightModeAltitudeHold)
            follower.AltitudeHold = base.Y

		// Match altitude using altitude hold PID
		follower.SetFlightMode(FlightModeAltitudeHold)
		altTarget := leader.Position.Y
		if s.hasLast {
			altTarget = s.last.Position.Y
		}
            follower.AltitudeHold = altTarget

		// Lateral control: PD on position error with relative velocity damping
		ex := targetPos.X - follower.Position.X
		ez := targetPos.Z - follower.Position.Z
		vxRel := follower.Velocity.X - lvel.X
		vzRel := follower.Velocity.Z - lvel.Z
        ax := kpPos*ex - kdVel*vxRel
        az := kpPos*ez - kdVel*vzRel
        // Limit commanded acceleration to avoid aggressive tilts
        maxAcc := 3.0
        if ax > maxAcc { ax = maxAcc } else if ax < -maxAcc { ax = -maxAcc }
        if az > maxAcc { az = maxAcc } else if az < -maxAcc { az = -maxAcc }
		// Convert desired acceleration to small-angle tilt targets
		pitchTarget := clamp(-math.Atan2(az, g), -10*math.Pi/180, 10*math.Pi/180)
		rollTarget := clamp(math.Atan2(ax, g), -10*math.Pi/180, 10*math.Pi/180)

		// Attitude PD tracking for pitch/roll
        torqueX := kpAtt*(pitchTarget-follower.Rotation.X) - kdAtt*follower.AngularVel.X
        torqueZ := kpAtt*(rollTarget-follower.Rotation.Z) - kdAtt*follower.AngularVel.Z
		// Align roll/pitch with leader attitude (leader typically level)
		kAlignAtt := 2.0
		torqueX += kAlignAtt * (lpitch - follower.Rotation.X)
		torqueZ += kAlignAtt * (lroll - follower.Rotation.Z)
		// Align yaw with leader
        yawErr := angleDiff(lyaw, follower.Rotation.Y)
        yawTorque := kpYaw*yawErr - kdYaw*follower.AngularVel.Y
		// Additional gate: require follower to have some altitude clearance before allowing lateral control
		followerClear := !follower.OnGround && (follower.Position.Y > follower.Dimensions.Z/2.0+0.2)
		active := allowFormation && followerClear
        // Additional altitude-based scaling to avoid tipping on/near ground
        agl := follower.Position.Y - follower.groundClearance()
        attScale := clamp((agl-0.05)/0.35, 0.0, 1.0)
        if attScale < 0 { attScale = 0 }
        torqueX *= attScale
        torqueZ *= attScale
        yawTorque *= clamp(attScale*1.2, 0.0, 1.0)

        // If formation is not yet allowed or follower lacks clearance, keep calm and avoid lateral/yaw commands
        if !active {
            torqueX = 0
            torqueZ = 0
            yawTorque = 0
            // Light lateral damping to prevent drift
            follower.Velocity.X *= 0.95
            follower.Velocity.Z *= 0.95
        }
        // Disable lateral torques while on ground to avoid skittering
        if follower.OnGround {
            torqueX = 0
            torqueZ = 0
        }
        // Saturate torques to avoid violent angular accelerations
        tMax := 0.35
        if torqueX > tMax { torqueX = tMax } else if torqueX < -tMax { torqueX = -tMax }
        if torqueZ > tMax { torqueZ = tMax } else if torqueZ < -tMax { torqueZ = -tMax }
        if yawTorque > 0.25 { yawTorque = 0.25 } else if yawTorque < -0.25 { yawTorque = -0.25 }
		follower.AddTorque(Vec3{X: torqueX, Y: yawTorque, Z: torqueZ}, dt)
		// Gentle clamp toward slot and velocity damping when far
		delta := follower.Position.Sub(targetPos)
		if active {
			if delta.Length() > 8.0 {
				follower.Position = follower.Position.Sub(delta.Mul(0.25))
				follower.Velocity = follower.Velocity.Mul(0.8)
			}
		} else {
			// While waiting for leader motion, only pull in if very far from leader
			if follower.Position.Sub(base).Length() > 20.0 {
				follower.Position = follower.Position.Sub(delta.Mul(0.15))
				follower.Velocity = follower.Velocity.Mul(0.85)
			}
		}
		// Hard leash to prevent runaway (always enabled)
		if follower.Position.Sub(base).Length() > 80.0 {
			follower.Position = targetPos
			follower.Velocity = Vec3{}
			follower.AngularVel = Vec3{}
		}
		rank++
	}
}

// initializeFollower sets safe initial conditions on arming.
// initializeFollower removed in simplified controller

// Reform snaps followers back to formation slots around the latest known leader position
// and resets their velocities/attitudes for recovery.
func (s *Swarm) Reform() {
	if len(s.drones) == 0 {
		return
	}
	leader := s.drones[s.leaderIdx]
	base := leader.Position
	if s.hasLast {
		base = s.last.Position
	}
	n := len(s.drones)
	followers := n - 1
	rank := 0
	R := 3.0
	for i := 0; i < n; i++ {
		if i == s.leaderIdx {
			continue
		}
		d := s.drones[i]
		angle := 0.0
		if followers > 0 {
			angle = 2 * math.Pi * float64(rank) / float64(followers)
		}
		target := base.Add(Vec3{R * math.Cos(angle), 0, R * math.Sin(angle)})
		d.Position = target
		d.Velocity = Vec3{}
		d.AngularVel = Vec3{}
		d.Rotation = Vec3{0, s.last.Yaw, 0}
        d.SetFlightMode(FlightModeAltitudeHold)
        d.AltitudeHold = base.Y
		rank++
	}
}

// MaxFollowerDistance returns the maximum distance of any follower from the leader.
func (s *Swarm) MaxFollowerDistance() float64 {
	if len(s.drones) == 0 {
		return 0
	}
	leader := s.drones[s.leaderIdx]
	maxd := 0.0
	for i, d := range s.drones {
		if i == s.leaderIdx {
			continue
		}
		dlen := d.Position.Sub(leader.Position).Length()
		if dlen > maxd {
			maxd = dlen
		}
	}
	return maxd
}

// MessageAge returns the time since the last leader state was delivered to followers.
func (s *Swarm) MessageAge() float64 {
	if !s.hasLast {
		return math.Inf(1)
	}
	// Next item in queue's delivery time minus current simTime doesn't help; we approximate
	// by latency for now when we have a last message.
	return s.latency
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

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

type LeaderState struct {
	Position   Vec3
	Velocity   Vec3
	Yaw        float64
	FlightMode FlightMode
	Throttle   float64
	IsArmed    bool
}

type scheduledMsg struct {
	deliverAt float64
	state     LeaderState
}

func angleDiff(target, current float64) float64 {
	d := target - current
	for d > math.Pi {
		d -= 2 * math.Pi
	}
	for d < -math.Pi {
		d += 2 * math.Pi
	}
	return d
}
