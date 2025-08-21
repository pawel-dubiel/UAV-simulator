package sim

import (
	"math"
)

type Vec3 struct {
	X, Y, Z float64
}

func (v Vec3) Add(other Vec3) Vec3 {
	return Vec3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vec3) Sub(other Vec3) Vec3 {
	return Vec3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vec3) Mul(scalar float64) Vec3 {
	return Vec3{v.X * scalar, v.Y * scalar, v.Z * scalar}
}

func (v Vec3) Dot(other Vec3) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

func (v Vec3) Cross(other Vec3) Vec3 {
	return Vec3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

func (v Vec3) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v Vec3) Normalize() Vec3 {
	length := v.Length()
	if length == 0 {
		return Vec3{0, 0, 0}
	}
	return Vec3{v.X / length, v.Y / length, v.Z / length}
}

type Mat4 [16]float32

func IdentityMat4() Mat4 {
	return Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

func PerspectiveMat4(fovy, aspect, near, far float64) Mat4 {
	f := 1.0 / math.Tan(fovy*math.Pi/360.0)
	nf := 1.0 / (near - far)

	// Column-major order for OpenGL
	return Mat4{
		float32(f / aspect), 0, 0, 0, // Column 0
		0, float32(f), 0, 0, // Column 1
		0, 0, float32((far + near) * nf), -1, // Column 2
		0, 0, float32(2 * far * near * nf), 0, // Column 3
	}
}

func LookAtMat4(eye, center, up Vec3) Mat4 {
	f := center.Sub(eye).Normalize()
	s := f.Cross(up).Normalize()
	u := s.Cross(f)

	// Column-major order for OpenGL
	return Mat4{
		float32(s.X), float32(s.Y), float32(s.Z), 0, // Column 0 (right)
		float32(u.X), float32(u.Y), float32(u.Z), 0, // Column 1 (up)
		float32(-f.X), float32(-f.Y), float32(-f.Z), 0, // Column 2 (forward)
		float32(-s.Dot(eye)), float32(-u.Dot(eye)), float32(f.Dot(eye)), 1, // Column 3 (translation)
	}
}

func TranslationMat4(v Vec3) Mat4 {
	// Column-major order for OpenGL
	return Mat4{
		1, 0, 0, 0, // Column 0
		0, 1, 0, 0, // Column 1
		0, 0, 1, 0, // Column 2
		float32(v.X), float32(v.Y), float32(v.Z), 1, // Column 3
	}
}

func RotationXMat4(angle float64) Mat4 {
	c := float32(math.Cos(angle))
	s := float32(math.Sin(angle))
	return Mat4{
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1,
	}
}

func RotationYMat4(angle float64) Mat4 {
	c := float32(math.Cos(angle))
	s := float32(math.Sin(angle))
	return Mat4{
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1,
	}
}

func RotationZMat4(angle float64) Mat4 {
	c := float32(math.Cos(angle))
	s := float32(math.Sin(angle))
	return Mat4{
		c, -s, 0, 0,
		s, c, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

func ScaleMat4(sx, sy, sz float64) Mat4 {
	return Mat4{
		float32(sx), 0, 0, 0,
		0, float32(sy), 0, 0,
		0, 0, float32(sz), 0,
		0, 0, 0, 1,
	}
}

func (m Mat4) Mul(other Mat4) Mat4 {
	// Column-major matrix multiplication: result = m * other
	// Indices are [col*4 + row]
	var result Mat4
	for col := 0; col < 4; col++ {
		for row := 0; row < 4; row++ {
			sum := float32(0.0)
			for k := 0; k < 4; k++ {
				sum += m[k*4+row] * other[col*4+k]
			}
			result[col*4+row] = sum
		}
	}
	return result
}
