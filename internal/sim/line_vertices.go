package sim

// LineVertex is a colored world-space vertex for debug line rendering.
type LineVertex struct {
	Position Vec3
	Color    Color
}

// BuildCommunicationLineVertices converts communication links to renderable line vertices.
func BuildCommunicationLineVertices(links []CommunicationLink) []LineVertex {
	verts := make([]LineVertex, 0, len(links)*2)
	for _, link := range links {
		color := communicationLinkColor(link.SnapshotAge, link.MaxAge)
		verts = append(verts,
			LineVertex{Position: link.ReceiverPosition, Color: color},
			LineVertex{Position: link.SenderSnapshotPosition, Color: color},
		)
	}
	return verts
}

func communicationLinkColor(age, maxAge float64) Color {
	ratio := 0.0
	if maxAge > 0 {
		ratio = age / maxAge
	}
	ratio = clamp(ratio, 0, 1)
	return Color{
		R: float32(ratio),
		G: 1,
		B: float32(1 - ratio),
		A: float32(1 - ratio*0.5),
	}
}
