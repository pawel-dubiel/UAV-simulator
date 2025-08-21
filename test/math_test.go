package sim_test

import (
	sim "drone-simulator/internal/sim"
	"testing"
)

func TestVec3Ops(t *testing.T) {
	a := sim.Vec3{X: 1, Y: 2, Z: 3}
	b := sim.Vec3{X: -3, Y: 0, Z: 5}
	if got := a.Add(b); got != (sim.Vec3{X: -2, Y: 2, Z: 8}) {
		t.Fatalf("Add got %v", got)
	}
	if got := a.Sub(b); got != (sim.Vec3{X: 4, Y: 2, Z: -2}) {
		t.Fatalf("Sub got %v", got)
	}
	if got := a.Mul(2); got != (sim.Vec3{X: 2, Y: 4, Z: 6}) {
		t.Fatalf("Mul got %v", got)
	}
	if a.Dot(b) != (1*-3 + 2*0 + 3*5) {
		t.Fatalf("Dot mismatch")
	}
	if l := a.Length(); l <= 0 {
		t.Fatalf("Length should be >0")
	}
	n := a.Normalize()
	if n.Length() < 0.99 || n.Length() > 1.01 {
		t.Fatalf("Normalize length ~1, got %v", n.Length())
	}
}
