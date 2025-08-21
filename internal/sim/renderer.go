//go:build !test
// +build !test

package sim

import (
	"fmt"
	"strings"

	"github.com/go-gl/gl/v4.1-core/gl"
)

const vertexShaderSource = `
#version 410 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 vertexColor;
out vec3 worldPos;

void main() {
    vec4 wp = model * vec4(aPos, 1.0);
    worldPos = wp.xyz;
    gl_Position = projection * view * wp;
    vertexColor = aColor;
}
` + "\x00"

const fragmentShaderSource = `
#version 410 core
in vec3 vertexColor;
in vec3 worldPos;
out vec4 FragColor;

uniform float uTileSize; // world units per tile
uniform vec3 uColorA;    // checker color A
uniform vec3 uColorB;    // checker color B
uniform int uUseChecker; // 1 = use checker, 0 = use vertexColor
uniform vec3 uCameraPos; // for fog based on distance
uniform vec3 uFogColor;
uniform float uFogDensity; // exponential fog density
uniform float uGridLineWidth; // world units
uniform vec3 uGridLineColor;
uniform float uGridLineAlpha; // 0..1

void main() {
    if (uUseChecker == 1) {
        // Checkerboard based on world XZ coordinates
        float tx = floor(worldPos.x / uTileSize);
        float tz = floor(worldPos.z / uTileSize);
        float checker = mod(tx + tz, 2.0);
        vec3 baseColor = mix(uColorA, uColorB, checker);

        // Subtle grid lines at tile boundaries
        float wx = worldPos.x / uTileSize;
        float wz = worldPos.z / uTileSize;
        float fx = abs(fract(wx) - 0.5);
        float fz = abs(fract(wz) - 0.5);
        float lineWidth = uGridLineWidth / uTileSize; // in tile fraction
        float lineMask = 1.0 - smoothstep(0.0, lineWidth, min(fx, fz));
        vec3 gridColor = mix(baseColor, uGridLineColor, lineMask * uGridLineAlpha);

        // Exponential fog by camera distance
        float dist = distance(worldPos, uCameraPos);
        float fogFactor = 1.0 - exp(-uFogDensity * dist);
        vec3 finalColor = mix(gridColor, uFogColor, clamp(fogFactor, 0.0, 1.0));
        FragColor = vec4(finalColor, 1.0);
    } else {
        FragColor = vec4(vertexColor, 1.0);
    }
}
` + "\x00"

type Renderer struct {
	shaderProgram uint32
	cubeVAO       uint32
	groundVAO     uint32
	modelLoc      int32
	viewLoc       int32
	projectionLoc int32
	tileSizeLoc   int32
	colorALoc     int32
	colorBLoc     int32
	useCheckerLoc int32
	cameraPosLoc  int32
	fogColorLoc   int32
	fogDensityLoc int32
	gridWidthLoc  int32
	gridColorLoc  int32
	gridAlphaLoc  int32

	cameraPos Vec3
}

func NewRenderer() *Renderer {
	r := &Renderer{}
	r.initShaders()
	r.initGeometry()
	return r
}

func (r *Renderer) initShaders() {
	vertexShader := compileShader(vertexShaderSource, gl.VERTEX_SHADER)
	fragmentShader := compileShader(fragmentShaderSource, gl.FRAGMENT_SHADER)

	r.shaderProgram = gl.CreateProgram()
	gl.AttachShader(r.shaderProgram, vertexShader)
	gl.AttachShader(r.shaderProgram, fragmentShader)
	gl.LinkProgram(r.shaderProgram)

	var success int32
	gl.GetProgramiv(r.shaderProgram, gl.LINK_STATUS, &success)
	if success == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(r.shaderProgram, gl.INFO_LOG_LENGTH, &logLength)
		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetProgramInfoLog(r.shaderProgram, logLength, nil, gl.Str(log))
		panic(fmt.Errorf("failed to link shader program: %v", log))
	}

	gl.DeleteShader(vertexShader)
	gl.DeleteShader(fragmentShader)

	r.modelLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("model\x00"))
	r.viewLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("view\x00"))
	r.projectionLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("projection\x00"))
	r.tileSizeLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uTileSize\x00"))
	r.colorALoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uColorA\x00"))
	r.colorBLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uColorB\x00"))
	r.useCheckerLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uUseChecker\x00"))
	r.cameraPosLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uCameraPos\x00"))
	r.fogColorLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uFogColor\x00"))
	r.fogDensityLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uFogDensity\x00"))
	r.gridWidthLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uGridLineWidth\x00"))
	r.gridColorLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uGridLineColor\x00"))
	r.gridAlphaLoc = gl.GetUniformLocation(r.shaderProgram, gl.Str("uGridLineAlpha\x00"))

	// Shader setup complete
}

func (r *Renderer) initGeometry() {
	// Proper 3D cube for drone
	cubeVertices := []float32{
		// Front face (red)
		-0.5, -0.2, 0.5, 1.0, 0.3, 0.3,
		0.5, -0.2, 0.5, 1.0, 0.3, 0.3,
		0.5, 0.2, 0.5, 1.0, 0.3, 0.3,
		-0.5, 0.2, 0.5, 1.0, 0.3, 0.3,
		// Back face (dark red)
		-0.5, -0.2, -0.5, 0.8, 0.2, 0.2,
		0.5, -0.2, -0.5, 0.8, 0.2, 0.2,
		0.5, 0.2, -0.5, 0.8, 0.2, 0.2,
		-0.5, 0.2, -0.5, 0.8, 0.2, 0.2,
	}

	// Debug output removed for clean startup

	cubeIndices := []uint32{
		// Front face
		0, 1, 2, 2, 3, 0,
		// Back face
		4, 5, 6, 6, 7, 4,
		// Left face
		7, 3, 0, 0, 4, 7,
		// Right face
		1, 5, 6, 6, 2, 1,
		// Top face
		3, 2, 6, 6, 7, 3,
		// Bottom face
		0, 1, 5, 5, 4, 0,
	}

	var cubeVBO, cubeEBO uint32
	gl.GenVertexArrays(1, &r.cubeVAO)
	gl.GenBuffers(1, &cubeVBO)
	gl.GenBuffers(1, &cubeEBO)

	gl.BindVertexArray(r.cubeVAO)
	gl.BindBuffer(gl.ARRAY_BUFFER, cubeVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(cubeVertices)*4, gl.Ptr(cubeVertices), gl.STATIC_DRAW)

	gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, cubeEBO)
	gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(cubeIndices)*4, gl.Ptr(cubeIndices), gl.STATIC_DRAW)

	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 6*4, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 6*4, gl.PtrOffset(3*4))
	gl.EnableVertexAttribArray(1)

	// Ground plane (large quad; shader draws checkerboard in world space)
	groundVertices := []float32{
		// Positions        Colors (green)
		-200.0, 0.0, -200.0, 0.3, 0.7, 0.3,
		200.0, 0.0, -200.0, 0.3, 0.7, 0.3,
		200.0, 0.0, 200.0, 0.3, 0.7, 0.3,
		-200.0, 0.0, 200.0, 0.3, 0.7, 0.3,
	}

	groundIndices := []uint32{
		0, 1, 2, 2, 3, 0,
	}

	var groundVBO, groundEBO uint32
	gl.GenVertexArrays(1, &r.groundVAO)
	gl.GenBuffers(1, &groundVBO)
	gl.GenBuffers(1, &groundEBO)

	gl.BindVertexArray(r.groundVAO)
	gl.BindBuffer(gl.ARRAY_BUFFER, groundVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(groundVertices)*4, gl.Ptr(groundVertices), gl.STATIC_DRAW)

	gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, groundEBO)
	gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(groundIndices)*4, gl.Ptr(groundIndices), gl.STATIC_DRAW)

	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 6*4, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 6*4, gl.PtrOffset(3*4))
	gl.EnableVertexAttribArray(1)
}

func (r *Renderer) SetMatrices(model, view, projection Mat4) {
	gl.UseProgram(r.shaderProgram)
	gl.UniformMatrix4fv(r.modelLoc, 1, false, &model[0])
	gl.UniformMatrix4fv(r.viewLoc, 1, false, &view[0])
	gl.UniformMatrix4fv(r.projectionLoc, 1, false, &projection[0])
}

func (r *Renderer) RenderDrone() {
	gl.BindVertexArray(r.cubeVAO)
	gl.UseProgram(r.shaderProgram)
	gl.Uniform1i(r.useCheckerLoc, 0)
	gl.DrawElements(gl.TRIANGLES, 36, gl.UNSIGNED_INT, gl.PtrOffset(0)) // 36 indices for full cube
	gl.BindVertexArray(0)
}

func (r *Renderer) RenderGround() {
	gl.BindVertexArray(r.groundVAO)
	// Set ground uniforms for checkerboard
	gl.UseProgram(r.shaderProgram)
	gl.Uniform1i(r.useCheckerLoc, 1)
	gl.Uniform1f(r.tileSizeLoc, 2.0)
	gl.Uniform3f(r.colorALoc, 0.28, 0.65, 0.28)
	gl.Uniform3f(r.colorBLoc, 0.24, 0.58, 0.24)
	gl.Uniform3f(r.cameraPosLoc, float32(r.cameraPos.X), float32(r.cameraPos.Y), float32(r.cameraPos.Z))
	gl.Uniform3f(r.fogColorLoc, 0.5, 0.7, 0.9)
	gl.Uniform1f(r.fogDensityLoc, 0.05)
	gl.Uniform1f(r.gridWidthLoc, 0.04)
	gl.Uniform3f(r.gridColorLoc, 0.18, 0.42, 0.18)
	gl.Uniform1f(r.gridAlphaLoc, 0.6)
	gl.DrawElements(gl.TRIANGLES, 6, gl.UNSIGNED_INT, gl.PtrOffset(0))
}

func (r *Renderer) SetCamera(pos Vec3) {
	r.cameraPos = pos
}

func compileShader(source string, shaderType uint32) uint32 {
	shader := gl.CreateShader(shaderType)
	cSources, free := gl.Strs(source)
	gl.ShaderSource(shader, 1, cSources, nil)
	free()
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)
		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))
		panic(fmt.Errorf("failed to compile %v: %v", source, log))
	}

	return shader
}
