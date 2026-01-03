//go:build !darwin && !test
// +build !darwin,!test

package avaudio

import "errors"

type System struct {
	droneCount int
}

func NewSystem(droneCount int, sampleRate int, samples []float32) (*System, error) {
	return nil, errors.New("audio init: AVAudioEngine is only supported on darwin")
}

func (s *System) DroneCount() int {
	if s == nil {
		return 0
	}
	return s.droneCount
}

func (s *System) SetListener(x, y, z, yawDeg, pitchDeg, rollDeg float64) error {
	return errors.New("audio update: AVAudioEngine is only supported on darwin")
}

func (s *System) SetSource(idx int, x, y, z float64, gain float32, rate float32) error {
	return errors.New("audio update: AVAudioEngine is only supported on darwin")
}

func (s *System) Close() {}
