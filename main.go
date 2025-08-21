//go:build !test
// +build !test

package main

import (
	"flag"
	"fmt"
	"log"
	"runtime"
	"time"

	sim "drone-simulator/internal/sim"
	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

func init() {
	runtime.LockOSThread()
}

func main() {
	// CLI flags
	headless := flag.Bool("headless", false, "Run without a window for benchmarking")
	steps := flag.Int("steps", 0, "Number of fixed updates to run in headless mode (0 = use duration)")
	duration := flag.Duration("duration", 0, "How long to run in headless mode (e.g., 5s); ignored if steps > 0")
    ups := flag.Int("ups", 120, "Fixed updates per second in headless mode")
    decoupled := flag.Bool("decoupled", true, "Run decoupled simulation/render loops (default true; pass -decoupled=false for legacy loop)")
	arm := flag.Bool("arm", true, "Auto-arm drones in headless mode")
	flag.Parse()

	if *headless {
		fmt.Println("Drone Simulator (headless benchmark) ...")
		s := sim.NewSimulatorHeadless()
		if *arm {
			for _, d := range s.Drones() {
				d.Arm()
				// Give a reasonable default throttle for activity
				d.SetThrottle(d.HoverThrottlePercent())
			}
		}
		start := time.Now()
		performed := s.RunHeadless(*steps, *ups, *duration)
		elapsed := time.Since(start)
		achievedUPS := float64(performed) / elapsed.Seconds()
		ad := s.ActiveDrone()
		fmt.Printf("Completed %d steps in %s (achieved ~%.1f UPS)\n", performed, elapsed.Truncate(time.Millisecond), achievedUPS)
		fmt.Printf("Leader pos=(%.2f, %.2f, %.2f) battery=%.1f%% throttle=%.0f%%\n", ad.Position.X, ad.Position.Y, ad.Position.Z, ad.BatteryPercent, ad.ThrottlePercent)
		return
	}

	fmt.Println("Drone Simulator Starting...")

	if err := glfw.Init(); err != nil {
		log.Fatal("Failed to initialize GLFW:", err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(1024, 768, "3D Drone Simulator", nil, nil)
	if err != nil {
		log.Fatal("Failed to create window:", err)
	}

	window.MakeContextCurrent()

	// Enable vsync for stable frame pacing
	glfw.SwapInterval(1)

	if err := gl.Init(); err != nil {
		log.Fatal("Failed to initialize OpenGL:", err)
	}

	gl.Viewport(0, 0, 1024, 768)
	gl.Enable(gl.DEPTH_TEST)
	gl.DepthFunc(gl.LESS)
	gl.Disable(gl.CULL_FACE)
	gl.PolygonMode(gl.FRONT_AND_BACK, gl.FILL)
	gl.ClearColor(0.5, 0.7, 0.9, 1.0) // Sky blue background
	// Enable blending for UI overlay transparency
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

	fmt.Printf("OpenGL version: %s\n", gl.GoStr(gl.GetString(gl.VERSION)))
	fmt.Printf("GLSL version: %s\n", gl.GoStr(gl.GetString(gl.SHADING_LANGUAGE_VERSION)))

    simulator := sim.NewSimulator()
    if *decoupled {
        simulator.RunDecoupled(window)
    } else {
        simulator.Run(window)
    }
}
