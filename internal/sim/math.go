package sim

import (
	"math"
)

type Vec3 struct {
	X, Y, Z float64
}

func (v Vec3) Add(other Vec3) Vec3     { return Vec3{v.X + other.X, v.Y + other.Y, v.Z + other.Z} }
func (v Vec3) Sub(other Vec3) Vec3     { return Vec3{v.X - other.X, v.Y - other.Y, v.Z - other.Z} }
func (v Vec3) Mul(scalar float64) Vec3 { return Vec3{v.X * scalar, v.Y * scalar, v.Z * scalar} }

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

func (v Vec3) Length() float64 { return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z) }

func (v Vec3) Normalize() Vec3 {
	length := v.Length()
	if length == 0 {
		return Vec3{0, 0, 0}
	}
	return Vec3{v.X / length, v.Y / length, v.Z / length}
}

// NormalizeSafe normalizes unless |v| < eps, in which case it returns (0,0,0).
func (v Vec3) NormalizeSafe(eps float64) Vec3 {
	if v.Length() < eps {
		return Vec3{0, 0, 0}
	}
	return v.Normalize()
}

type Vec4 struct {
	X, Y, Z, W float64
}

func DegToRad(deg float64) float64 { return deg * math.Pi / 180.0 }
func RadToDeg(rad float64) float64 { return rad * 180.0 / math.Pi }

// Mat4 is a 4x4 matrix in column-major order (OpenGL-style).
type Mat4 [16]float64

func IdentityMat4() Mat4 {
	return Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

// PerspectiveMat4 builds a right-handed OpenGL projection.
// fovy is in DEGREES, aspect = width/height, NDC z ∈ [-1,1].
func PerspectiveMat4(fovy, aspect, near, far float64) Mat4 {
	f := 1.0 / math.Tan(fovy*math.Pi/360.0) // = 1 / tan(fovy/2)
	nf := 1.0 / (near - far)

	return Mat4{
		f / aspect, 0, 0, 0,
		0, f, 0, 0,
		0, 0, (far + near) * nf, -1,
		0, 0, 2 * far * near * nf, 0,
	}
}

// LookAtMat4 creates a right-handed view matrix.
func LookAtMat4(eye, center, up Vec3) Mat4 {
	const eps = 1e-8

	f := center.Sub(eye).NormalizeSafe(eps)
	s := f.Cross(up)
	if s.Length() < eps {
		// Choose an alternate up if too parallel
		var altUp Vec3
		if math.Abs(f.X) < 0.9 {
			altUp = Vec3{1, 0, 0}
		} else {
			altUp = Vec3{0, 1, 0}
		}
		s = f.Cross(altUp)
	}
	s = s.Normalize()
	u := s.Cross(f)

	return Mat4{
		s.X, s.Y, s.Z, 0,
		u.X, u.Y, u.Z, 0,
		-f.X, -f.Y, -f.Z, 0,
		-s.Dot(eye), -u.Dot(eye), f.Dot(eye), 1,
	}
}

func TranslationMat4(v Vec3) Mat4 {
	return Mat4{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		v.X, v.Y, v.Z, 1,
	}
}

func RotationXMat4(angle float64) Mat4 {
	c := math.Cos(angle)
	s := math.Sin(angle)
	return Mat4{
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1,
	}
}

func RotationYMat4(angle float64) Mat4 {
	c := math.Cos(angle)
	s := math.Sin(angle)
	return Mat4{
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1,
	}
}

func RotationZMat4(angle float64) Mat4 {
	c := math.Cos(angle)
	s := math.Sin(angle)
	return Mat4{
		c, -s, 0, 0,
		s, c, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

func ScaleMat4(sx, sy, sz float64) Mat4 {
	return Mat4{
		sx, 0, 0, 0,
		0, sy, 0, 0,
		0, 0, sz, 0,
		0, 0, 0, 1,
	}
}

// Mul performs column-major matrix multiplication: result = m * other.
func (m Mat4) Mul(other Mat4) Mat4 {
	var result Mat4
	for col := 0; col < 4; col++ {
		for row := 0; row < 4; row++ {
			sum := 0.0
			for k := 0; k < 4; k++ {
				sum += m[k*4+row] * other[col*4+k]
			}
			result[col*4+row] = sum
		}
	}
	return result
}

// MulVec4 multiplies matrix by a column vector v: r = m * v.
func (m Mat4) MulVec4(v Vec4) Vec4 {
	r0 := m[0*4+0]*v.X + m[1*4+0]*v.Y + m[2*4+0]*v.Z + m[3*4+0]*v.W
	r1 := m[0*4+1]*v.X + m[1*4+1]*v.Y + m[2*4+1]*v.Z + m[3*4+1]*v.W
	r2 := m[0*4+2]*v.X + m[1*4+2]*v.Y + m[2*4+2]*v.Z + m[3*4+2]*v.W
	r3 := m[0*4+3]*v.X + m[1*4+3]*v.Y + m[2*4+3]*v.Z + m[3*4+3]*v.W
	return Vec4{r0, r1, r2, r3}
}

// MulPoint transforms a point (w=1) and applies perspective divide if w != 0.
func (m Mat4) MulPoint(p Vec3) Vec3 {
	r := m.MulVec4(Vec4{p.X, p.Y, p.Z, 1})
	if r.W != 0 {
		inv := 1.0 / r.W
		return Vec3{r.X * inv, r.Y * inv, r.Z * inv}
	}
	return Vec3{r.X, r.Y, r.Z}
}

// MulDirection transforms a direction (w=0), ignoring translation.
func (m Mat4) MulDirection(d Vec3) Vec3 {
	r := m.MulVec4(Vec4{d.X, d.Y, d.Z, 0})
	return Vec3{r.X, r.Y, r.Z}
}
