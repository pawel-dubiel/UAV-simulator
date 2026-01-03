//go:build !test
// +build !test

package sim

import (
	"errors"
	"fmt"
	"math"

	"drone-simulator/internal/sim/avaudio"
)

const (
	audioSampleRate  = 48000
	audioRotorBlades = 2
	audioRotorCycles = 128
	audioNominalRPM  = 4000.0
	audioMaxRPM      = 8000.0
)

type AudioSystem struct {
	backend *avaudio.System
	baseRPM float64
	closed  bool
}

func NewAudioSystem(droneCount int) (*AudioSystem, error) {
	if droneCount <= 0 {
		return nil, errors.New("audio init: drone count must be > 0")
	}

	data, baseRPM, err := synthRotorSample(audioSampleRate, audioRotorBlades, audioRotorCycles, audioNominalRPM)
	if err != nil {
		return nil, err
	}
	backend, err := avaudio.NewSystem(droneCount, audioSampleRate, data)
	if err != nil {
		return nil, fmt.Errorf("audio init: %w", err)
	}

	return &AudioSystem{
		backend: backend,
		baseRPM: baseRPM,
	}, nil
}

func (a *AudioSystem) Update(camera *Camera, drones []*Drone) error {
	if a == nil || a.closed {
		return errors.New("audio update: audio system not initialized")
	}
	if camera == nil {
		return errors.New("audio update: camera is nil")
	}
	if len(drones) != a.backend.DroneCount() {
		return fmt.Errorf("audio update: drone count %d does not match sources %d", len(drones), a.backend.DroneCount())
	}

	forward := camera.Target.Sub(camera.Position)
	if forward.Length() <= 1e-6 {
		return errors.New("audio update: camera forward vector is zero")
	}
	forward = forward.Normalize()

	yawDeg := math.Atan2(forward.X, forward.Z) * 180.0 / math.Pi
	pitchDeg := -math.Asin(forward.Y) * 180.0 / math.Pi
	if err := a.backend.SetListener(camera.Position.X, camera.Position.Y, camera.Position.Z, yawDeg, pitchDeg, 0); err != nil {
		return err
	}

	for i, d := range drones {
		if d == nil {
			return fmt.Errorf("audio update: drone %d is nil", i)
		}
		if d.lastMaxVerticalThrustN <= 0 {
			return fmt.Errorf("audio update: drone %d has non-positive max vertical thrust", i)
		}
		norm := d.lastVerticalThrustN / d.lastMaxVerticalThrustN
		if norm < 0 {
			norm = 0
		}
		if norm > 1 {
			norm = 1
		}
		effectiveRPM := audioMaxRPM * math.Sqrt(norm)
		gain := rpmGain(effectiveRPM, d.IsArmed)
		rate := rpmRate(effectiveRPM, a.baseRPM, d.IsArmed)
		if err := a.backend.SetSource(i, d.Position.X, d.Position.Y, d.Position.Z, gain, rate); err != nil {
			return err
		}
	}
	return nil
}

func (a *AudioSystem) Close() {
	if a == nil || a.closed {
		return
	}
	if a.backend != nil {
		a.backend.Close()
	}
	a.closed = true
}

func averageRPM(rpms []float64) float64 {
	if len(rpms) == 0 {
		return 0
	}
	sum := 0.0
	for _, rpm := range rpms {
		if rpm < 0 {
			rpm = -rpm
		}
		sum += rpm
	}
	return sum / float64(len(rpms))
}

func rpmGain(rpm float64, armed bool) float32 {
	if !armed || rpm <= 1 {
		return 0
	}
	norm := rpm / audioMaxRPM
	if norm < 0 {
		norm = 0
	}
	if norm > 1 {
		norm = 1
	}
	return float32(0.08 + 0.6*norm)
}

func rpmRate(rpm, baseRPM float64, armed bool) float32 {
	if !armed || rpm <= 1 || baseRPM <= 0 {
		return 0.01
	}
	ratio := rpm / baseRPM
	if ratio < 0.4 {
		ratio = 0.4
	}
	if ratio > 2.2 {
		ratio = 2.2
	}
	return float32(ratio)
}

func synthRotorSample(sampleRate, blades, cycles int, nominalRPM float64) ([]float32, float64, error) {
	if sampleRate <= 0 {
		return nil, 0, errors.New("audio init: sample rate must be > 0")
	}
	if blades <= 0 {
		return nil, 0, errors.New("audio init: blade count must be > 0")
	}
	if cycles <= 0 {
		return nil, 0, errors.New("audio init: cycles must be > 0")
	}
	if nominalRPM <= 0 {
		return nil, 0, errors.New("audio init: nominal RPM must be > 0")
	}

	baseFreq := nominalRPM / 60.0 * float64(blades)
	sampleCount := int(math.Round(float64(cycles) * float64(sampleRate) / baseFreq))
	if sampleCount < 1 {
		return nil, 0, errors.New("audio init: sample count too small")
	}

	baseFreq = float64(cycles) * float64(sampleRate) / float64(sampleCount)
	baseRPM := baseFreq * 60.0 / float64(blades)

	samples := make([]float64, sampleCount)
	phaseStep := 2.0 * math.Pi * baseFreq / float64(sampleRate)
	for i := 0; i < sampleCount; i++ {
		phase := float64(i) * phaseStep
		mod := 0.15 * math.Sin(phase*0.5)
		tone := math.Sin(phase) + 0.35*math.Sin(2*phase+0.1) + 0.2*math.Sin(3*phase+0.2)
		samples[i] = tone * (0.7 + mod)
	}

	maxAbs := 0.0
	for _, v := range samples {
		if v < 0 {
			v = -v
		}
		if v > maxAbs {
			maxAbs = v
		}
	}
	if maxAbs <= 0 {
		return nil, 0, errors.New("audio init: sample amplitude is zero")
	}

	scale := 0.85 / maxAbs
	data := make([]float32, sampleCount)
	for i, v := range samples {
		data[i] = float32(v * scale)
	}

	return data, baseRPM, nil
}
