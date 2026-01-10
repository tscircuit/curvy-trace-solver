import type { Point } from "@tscircuit/math-utils"

/**
 * Samples a trace (array of points) into evenly-spaced segments.
 * This is useful for limiting computation when comparing traces.
 */
export const sampleTraceIntoSegments = (
  points: Point[],
  numSegments: number,
  networkId?: string,
): { segment: [Point, Point]; networkId?: string }[] => {
  if (points.length < 2) return []

  // Compute total length of the trace
  let totalLength = 0
  for (let i = 0; i < points.length - 1; i++) {
    const dx = points[i + 1].x - points[i].x
    const dy = points[i + 1].y - points[i].y
    totalLength += Math.sqrt(dx * dx + dy * dy)
  }

  if (totalLength === 0) return []

  const segmentLength = totalLength / numSegments
  const sampledPoints: Point[] = [points[0]]

  let currentLength = 0
  let pointIndex = 0

  for (let i = 1; i <= numSegments; i++) {
    const targetLength = i * segmentLength

    while (pointIndex < points.length - 1) {
      const dx = points[pointIndex + 1].x - points[pointIndex].x
      const dy = points[pointIndex + 1].y - points[pointIndex].y
      const segLen = Math.sqrt(dx * dx + dy * dy)

      if (currentLength + segLen >= targetLength) {
        const t = (targetLength - currentLength) / segLen
        sampledPoints.push({
          x: points[pointIndex].x + t * dx,
          y: points[pointIndex].y + t * dy,
        })
        break
      }

      currentLength += segLen
      pointIndex++
    }
  }

  // Convert sampled points to segments
  const segments: { segment: [Point, Point]; networkId?: string }[] = []
  for (let i = 0; i < sampledPoints.length - 1; i++) {
    segments.push({
      segment: [sampledPoints[i], sampledPoints[i + 1]],
      networkId,
    })
  }

  return segments
}
