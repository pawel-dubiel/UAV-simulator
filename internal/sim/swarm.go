package sim

import (
    "math"
)

// Swarm coordinates distributed, decentralized behavior using local rules.
// Drones remain physically independent; Swarm orchestrates neighbor discovery,
// rule weights, and group-level utilities like re-forming.
type Swarm struct {
    drones []*Drone
    simTime float64
    // per-drone control memory for smoothing targets
    ctrl map[int]*followerCtrlState

    // Distributed (boids) parameters
    sensorRange       float64 // neighbor sensing range (m)
    separationRadius  float64 // m
    alignmentRadius   float64 // m
    cohesionRadius    float64 // m
    weightSeparation  float64
    weightAlignment   float64
    weightCohesion    float64
    weightGoal        float64
    maxAcc            float64 // clamp lateral commanded acceleration (m/s^2)
    maxTiltRateRad    float64 // rad/s slew limit for targets
    desiredAlt        float64 // shared target altitude (m); <=0 means keep current
    goal              Vec3    // optional global goal to drift toward
}

func NewSwarm(drones []*Drone) *Swarm {
    s := &Swarm{
        drones:   drones,
        ctrl:     make(map[int]*followerCtrlState),

        // Distributed defaults (tuned conservatively for stability)
        sensorRange:      25.0,
        separationRadius: 1.5,
        alignmentRadius:  4.0,
        cohesionRadius:   6.0,
        weightSeparation: 2.0,
        weightAlignment:  1.0,
        weightCohesion:   0.8,
        weightGoal:       0.6,
        maxAcc:           3.0,
        maxTiltRateRad:   120 * math.Pi / 180.0,
        desiredAlt:       0, // 0 => keep current per drone on first update
        goal:             Vec3{},
    }
    return s
}

func (s *Swarm) Update(dt float64) {
    if len(s.drones) == 0 {
        return
    }
    s.updateDistributedBoids(dt)
}

// --- Distributed Boids implementation ---

// neighborInfo is a lightweight view of a neighbor for boids rules.
type neighborInfo struct {
    idx int
    d   *Drone
    dist float64
}

func (s *Swarm) updateDistributedBoids(dt float64) {
    s.simTime += dt
    n := len(s.drones)
    if n == 0 { return }

    // Arm/disarm is not leader-driven in this mode; keep current states.
    // Ensure altitude hold targets are initialized if desiredAlt <= 0.
    for i, d := range s.drones {
        st := s.ctrl[i]
        if st == nil { st = &followerCtrlState{}; s.ctrl[i] = st }
        // Initialize altitude hold once on first tick
        if st.prevInit == 0 {
            if s.desiredAlt > 0 {
                d.SetFlightMode(FlightModeAltitudeHold)
                d.AltitudeHold = s.desiredAlt
            } else if d.FlightMode != FlightModeAltitudeHold && d.FlightMode != FlightModeHover {
                d.SetFlightMode(FlightModeAltitudeHold)
                d.AltitudeHold = d.Position.Y
            }
            st.prevPitch = d.Rotation.X
            st.prevRoll = d.Rotation.Z
            st.prevInit = 1
        }
    }

    // For each drone, compute local neighbor-based forces
    g := 9.81
    for i, d := range s.drones {
        // Skip unarmed drones: keep them calm on ground
        if !d.IsArmed {
            continue
        }
        // Per-drone control memory
        st := s.ctrl[i]
        if st == nil { st = &followerCtrlState{}; s.ctrl[i] = st }

        // Gather neighbors within sensor range (exclude self)
        neigh := make([]neighborInfo, 0, 8)
        for j, o := range s.drones {
            if j == i { continue }
            delta := o.Position.Sub(d.Position)
            dist := delta.Length()
            if dist <= s.sensorRange {
                neigh = append(neigh, neighborInfo{idx: j, d: o, dist: dist})
            }
        }

        // Accumulators
        sep := Vec3{}
        ali := Vec3{}
        coh := Vec3{}
        countAli := 0
        countCoh := 0
        // Separation: push away if too close
        for _, nfo := range neigh {
            if nfo.dist < 1e-6 { continue }
            if nfo.dist < s.separationRadius {
                away := d.Position.Sub(nfo.d.Position).Mul(1.0 / nfo.dist)
                // Weight more when closer
                factor := (s.separationRadius - nfo.dist) / s.separationRadius
                sep = sep.Add(away.Mul(factor))
            }
            if nfo.dist < s.alignmentRadius {
                ali = ali.Add(nfo.d.Velocity)
                countAli++
            }
            if nfo.dist < s.cohesionRadius {
                coh = coh.Add(nfo.d.Position)
                countCoh++
            }
        }
        if countAli > 0 {
            ali = ali.Mul(1.0 / float64(countAli))
            // We want to align horizontally; ignore vertical component
            ali.Y = 0
            // Steering toward average neighbor velocity
            ali = ali.Sub(d.Velocity)
        }
        if countCoh > 0 {
            cm := coh.Mul(1.0 / float64(countCoh))
            toCenter := cm.Sub(d.Position)
            toCenter.Y = 0
            // steer toward local center
            coh = toCenter
        } else {
            coh = Vec3{}
        }

        // Optional goal seeking (global drift)
        goal := Vec3{}
        if (s.goal.X != 0 || s.goal.Y != 0 || s.goal.Z != 0) {
            toGoal := s.goal.Sub(d.Position)
            toGoal.Y = 0
            goal = toGoal
        }

        // Combine forces and map to desired horizontal acceleration
        acc := sep.Mul(s.weightSeparation).
            Add(ali.Mul(s.weightAlignment)).
            Add(coh.Mul(s.weightCohesion)).
            Add(goal.Mul(s.weightGoal))

        // Convert to acceleration-like command by scaling and clamp magnitude
        // Use simple proportional factor to keep magnitudes reasonable
        acc = acc.Mul(1.0)
        // Limit command
        ax := acc.X
        az := acc.Z
        if ax > s.maxAcc { ax = s.maxAcc } else if ax < -s.maxAcc { ax = -s.maxAcc }
        if az > s.maxAcc { az = s.maxAcc } else if az < -s.maxAcc { az = -s.maxAcc }

        // Map to tilt targets (small-angle): pitch controls forward (Z), roll controls right (X)
        pitchCmd := clamp(-math.Atan2(az, g), -12*math.Pi/180, 12*math.Pi/180)
        rollCmd  := clamp( math.Atan2(ax, g), -12*math.Pi/180, 12*math.Pi/180)

        pitchTarget := slew(st.prevPitch, pitchCmd, s.maxTiltRateRad, dt)
        rollTarget  := slew(st.prevRoll,  rollCmd,  s.maxTiltRateRad, dt)
        st.prevPitch = pitchTarget
        st.prevRoll  = rollTarget

        // PD attitude tracking
        kpAtt := 4.0
        kdAtt := 3.0
        torqueX := kpAtt*(pitchTarget-d.Rotation.X) - kdAtt*d.AngularVel.X
        torqueZ := kpAtt*(rollTarget-d.Rotation.Z) - kdAtt*d.AngularVel.Z

        // Yaw aligns with velocity heading if moving
        yawTorque := 0.0
        v := d.Velocity
        if v.X*v.X+v.Z*v.Z > 0.25 { // >0.5 m/s
            desiredYaw := math.Atan2(v.X, v.Z) // note: X right, Z forward
            yawErr := angleDiff(desiredYaw, d.Rotation.Y)
            kpYaw := 1.0
            kdYaw := 0.7
            yawTorque = kpYaw*yawErr - kdYaw*d.AngularVel.Y
        }

        // Attitude scaling near ground to avoid tipping
        agl := d.Position.Y - d.groundClearance()
        attScale := clamp((agl-0.05)/0.35, 0.0, 1.0)
        torqueX *= attScale
        torqueZ *= attScale
        yawTorque *= clamp(attScale*1.2, 0.0, 1.0)

        // Saturation
        tMax := 0.35
        if torqueX > tMax { torqueX = tMax } else if torqueX < -tMax { torqueX = -tMax }
        if torqueZ > tMax { torqueZ = tMax } else if torqueZ < -tMax { torqueZ = -tMax }
        if yawTorque > 0.25 { yawTorque = 0.25 } else if yawTorque < -0.25 { yawTorque = -0.25 }
        d.AddTorque(Vec3{X: torqueX, Y: yawTorque, Z: torqueZ}, dt)
    }
}

// followerCtrlState holds per-follower control memory for smoothing
type followerCtrlState struct {
    prevPitch float64
    prevRoll  float64
    prevInit  int // 0=not init, 1=init
}

// initializeFollower sets safe initial conditions on arming.
// initializeFollower removed in simplified controller

// Reform repositions the swarm into a compact ring around the current
// centroid and resets velocities/attitudes for recovery.
func (s *Swarm) Reform() {
    if len(s.drones) == 0 {
        return
    }
    // Compute centroid of current positions
    base := Vec3{}
    for _, d := range s.drones { base = base.Add(d.Position) }
    inv := 1.0 / float64(len(s.drones))
    base = base.Mul(inv)
    n := len(s.drones)
    R := 3.0
    for i := 0; i < n; i++ {
        d := s.drones[i]
        angle := 2 * math.Pi * float64(i) / float64(n)
        target := base.Add(Vec3{R * math.Cos(angle), 0, R * math.Sin(angle)})
        d.Position = target
        d.Velocity = Vec3{}
        d.AngularVel = Vec3{}
        d.Rotation = Vec3{}
        d.SetFlightMode(FlightModeAltitudeHold)
        d.AltitudeHold = base.Y
    }
}

// MaxDistanceFromCentroid returns the maximum distance of any drone from swarm centroid.
func (s *Swarm) MaxDistanceFromCentroid() float64 {
    if len(s.drones) == 0 { return 0 }
    cen := Vec3{}
    for _, d := range s.drones { cen = cen.Add(d.Position) }
    cen = cen.Mul(1.0/float64(len(s.drones)))
    maxd := 0.0
    for _, d := range s.drones {
        dlen := d.Position.Sub(cen).Length()
        if dlen > maxd { maxd = dlen }
    }
    return maxd
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
