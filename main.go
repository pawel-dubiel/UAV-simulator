package main

import (
	"fmt"
	"log"
	"runtime"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

func init() {
	runtime.LockOSThread()
}

func main() {
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
	
	if err := gl.Init(); err != nil {
		log.Fatal("Failed to initialize OpenGL:", err)
	}

	gl.Viewport(0, 0, 1024, 768)
	gl.Enable(gl.DEPTH_TEST)
	gl.DepthFunc(gl.LESS)
	gl.Disable(gl.CULL_FACE)
	gl.PolygonMode(gl.FRONT_AND_BACK, gl.FILL)
	gl.ClearColor(0.5, 0.7, 0.9, 1.0)  // Sky blue background
	
	fmt.Printf("OpenGL version: %s\n", gl.GoStr(gl.GetString(gl.VERSION)))
	fmt.Printf("GLSL version: %s\n", gl.GoStr(gl.GetString(gl.SHADING_LANGUAGE_VERSION)))

	simulator := NewSimulator()
	simulator.Run(window)
}