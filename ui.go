package main

import (
    "strings"

    "github.com/go-gl/gl/v4.1-core/gl"
)

type Color struct { R, G, B, A float32 }

type UIRenderer struct {
    shader uint32
    vao    uint32
    vbo    uint32
    verts  []float32 // x,y,r,g,b,a per-vertex
    scrW   int
    scrH   int
}

func NewUIRenderer() *UIRenderer {
    ui := &UIRenderer{}
    ui.init()
    return ui
}

func (u *UIRenderer) init() {
    // Simple shader for 2D colored rectangles in NDC
    vs := `#version 410 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec4 aColor;
out vec4 vColor;
void main(){
    gl_Position = vec4(aPos, 0.0, 1.0);
    vColor = aColor;
}` + "\x00"
    fs := `#version 410 core
in vec4 vColor;
out vec4 FragColor;
void main(){
    FragColor = vColor;
}` + "\x00"

    v := compileShader(vs, gl.VERTEX_SHADER)
    f := compileShader(fs, gl.FRAGMENT_SHADER)
    u.shader = gl.CreateProgram()
    gl.AttachShader(u.shader, v)
    gl.AttachShader(u.shader, f)
    gl.LinkProgram(u.shader)
    gl.DeleteShader(v)
    gl.DeleteShader(f)

    var status int32
    gl.GetProgramiv(u.shader, gl.LINK_STATUS, &status)
    if status == gl.FALSE {
        var logLen int32
        gl.GetProgramiv(u.shader, gl.INFO_LOG_LENGTH, &logLen)
        log := strings.Repeat("\x00", int(logLen+1))
        gl.GetProgramInfoLog(u.shader, logLen, nil, gl.Str(log))
        panic("UI shader link failed: " + log)
    }

    gl.GenVertexArrays(1, &u.vao)
    gl.GenBuffers(1, &u.vbo)
    gl.BindVertexArray(u.vao)
    gl.BindBuffer(gl.ARRAY_BUFFER, u.vbo)
    gl.BufferData(gl.ARRAY_BUFFER, 0, nil, gl.DYNAMIC_DRAW)
    gl.VertexAttribPointer(0, 2, gl.FLOAT, false, 6*4, gl.PtrOffset(0))
    gl.EnableVertexAttribArray(0)
    gl.VertexAttribPointer(1, 4, gl.FLOAT, false, 6*4, gl.PtrOffset(2*4))
    gl.EnableVertexAttribArray(1)
}

func (u *UIRenderer) Begin(width, height int) {
    u.scrW, u.scrH = width, height
    u.verts = u.verts[:0]
}

func (u *UIRenderer) Flush() {
    if len(u.verts) == 0 { return }
    gl.UseProgram(u.shader)
    gl.BindVertexArray(u.vao)
    gl.BindBuffer(gl.ARRAY_BUFFER, u.vbo)
    gl.BufferData(gl.ARRAY_BUFFER, len(u.verts)*4, gl.Ptr(u.verts), gl.DYNAMIC_DRAW)
    gl.DrawArrays(gl.TRIANGLES, 0, int32(len(u.verts)/6))
    gl.BindVertexArray(0)
}

func (u *UIRenderer) AddRect(x, y, w, h int, c Color) {
    // Pixel top-left coords to NDC
    x0 := u.pxToNDCX(float32(x))
    y0 := u.pxToNDCY(float32(y))
    x1 := u.pxToNDCX(float32(x+w))
    y1 := u.pxToNDCY(float32(y+h))
    // Two triangles
    u.addV(x0, y0, c)
    u.addV(x1, y0, c)
    u.addV(x1, y1, c)

    u.addV(x0, y0, c)
    u.addV(x1, y1, c)
    u.addV(x0, y1, c)
}

func (u *UIRenderer) addV(x, y float32, c Color) {
    u.verts = append(u.verts, x, y, c.R, c.G, c.B, c.A)
}

func (u *UIRenderer) pxToNDCX(px float32) float32 {
    return (px/float32(u.scrW))*2 - 1
}

func (u *UIRenderer) pxToNDCY(py float32) float32 {
    // Convert from top-left origin pixels to NDC
    return 1 - (py/float32(u.scrH))*2
}

// 5x7 uppercase font for minimal HUD text
// Each row is 5 LSBits used.
var font5x7 = map[rune][7]uint8{
    ' ': {0,0,0,0,0,0,0},
    '.': {0,0,0,0,0,0,0b00100},
    ':': {0,0,0b010,0,0b010,0,0},
    '%': {0b10001,0b00010,0b00100,0b01000,0b10000,0,0},
    '-': {0,0,0b11110,0,0,0,0},

    '0': {0b01110,0b10001,0b10011,0b10101,0b11001,0b10001,0b01110},
    '1': {0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110},
    '2': {0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111},
    '3': {0b11110,0b00001,0b00001,0b01110,0b00001,0b00001,0b11110},
    '4': {0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010},
    '5': {0b11111,0b10000,0b11110,0b00001,0b00001,0b10001,0b01110},
    '6': {0b00110,0b01000,0b10000,0b11110,0b10001,0b10001,0b01110},
    '7': {0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000},
    '8': {0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110},
    '9': {0b01110,0b10001,0b10001,0b01111,0b00001,0b00010,0b01100},

    'A': {0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001},
    'B': {0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110},
    'C': {0b01110,0b10001,0b10000,0b10000,0b10000,0b10001,0b01110},
    'D': {0b11100,0b10010,0b10001,0b10001,0b10001,0b10010,0b11100},
    'E': {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111},
    'F': {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000},
    'G': {0b01110,0b10001,0b10000,0b10111,0b10001,0b10001,0b01110},
    'H': {0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001},
    'I': {0b01110,0b00100,0b00100,0b00100,0b00100,0b00100,0b01110},
    'J': {0b00001,0b00001,0b00001,0b00001,0b10001,0b10001,0b01110},
    'K': {0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001},
    'L': {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111},
    'M': {0b10001,0b11011,0b10101,0b10101,0b10001,0b10001,0b10001},
    'N': {0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001},
    'O': {0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110},
    'P': {0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000},
    'Q': {0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101},
    'R': {0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001},
    'S': {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110},
    'T': {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100},
    'U': {0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110},
    'V': {0b10001,0b10001,0b10001,0b10001,0b01010,0b01010,0b00100},
    'W': {0b10001,0b10001,0b10001,0b10101,0b10101,0b11011,0b10001},
    'X': {0b10001,0b10001,0b01010,0b00100,0b01010,0b10001,0b10001},
    'Y': {0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100},
    'Z': {0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111},
}

// DrawText draws uppercase-only text with a minimal 5x7 font.
// scale is the pixel size of one font pixel.
func (u *UIRenderer) DrawText(x, y int, text string, scale int, c Color) {
    // Ensure uppercase for glyph lookup
    s := strings.ToUpper(text)
    cx := x
    cw := 5 * scale
    ch := 7 * scale
    for _, r := range s {
        if r == '\n' {
            y += ch + scale
            cx = x
            continue
        }
        glyph, ok := font5x7[r]
        if !ok {
            // Fallback to space width
            cx += cw + scale
            continue
        }
        for row := 0; row < 7; row++ {
            bits := glyph[row]
            for col := 0; col < 5; col++ {
                if (bits&(1<<uint(4-col))) != 0 { // left-most is MSB among 5 bits
                    u.AddRect(cx+col*scale, y+row*scale, scale, scale, c)
                }
            }
        }
        cx += cw + scale
    }
}
