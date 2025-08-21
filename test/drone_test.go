package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestThrottleClamp(t *testing.T) {
	d := sim.NewDrone()
	d.SetThrottle(-10)
	if d.ThrottlePercent != 0 {
		t.Fatalf("expected 0, got %v", d.ThrottlePercent)
	}
	d.SetThrottle(150)
	if d.ThrottlePercent != 100 {
		t.Fatalf("expected 100, got %v", d.ThrottlePercent)
	}
}

func TestArmDisarm(t *testing.T) {
	d := sim.NewDrone()
	if d.IsArmed {
		t.Fatalf("expected disarmed")
	}
	d.Arm()
	if !d.IsArmed {
		t.Fatalf("expected armed")
	}
	d.Disarm()
	if d.IsArmed {
		t.Fatalf("expected disarmed after Disarm()")
	}
}

func TestHoverThrottle(t *testing.T) {
	d := sim.NewDrone()
	ht := d.HoverThrottlePercent()
	if ht <= 0 || ht >= 100 {
		t.Fatalf("hover throttle out of range: %v", ht)
	}
}
