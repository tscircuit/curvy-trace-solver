import { expect, test } from "bun:test"
import { AngledTraceSolver } from "lib/AngledTraceSolver"
import { generateRandomProblem } from "lib/problem-generator"
import type { CurvyTraceProblem } from "lib/types"

/**
 * Check if a segment is at a valid angle (horizontal, vertical, or 45°)
 * Returns the angle in degrees for debugging
 */
function getSegmentAngle(
  x1: number,
  y1: number,
  x2: number,
  y2: number,
): { valid: boolean; angle: number; dx: number; dy: number } {
  const dx = x2 - x1
  const dy = y2 - y1
  const absDx = Math.abs(dx)
  const absDy = Math.abs(dy)
  const eps = 1e-3 // Use larger epsilon for floating point comparison

  // Calculate angle in degrees
  const angle = (Math.atan2(dy, dx) * 180) / Math.PI

  // Horizontal: dy ≈ 0
  if (absDy < eps) return { valid: true, angle, dx, dy }
  // Vertical: dx ≈ 0
  if (absDx < eps) return { valid: true, angle, dx, dy }
  // 45°: |dx| ≈ |dy|
  if (Math.abs(absDx - absDy) < eps) return { valid: true, angle, dx, dy }

  return { valid: false, angle, dx, dy }
}

test("angled-trace-solver: all segments should be at valid 45° angles", () => {
  const problem = generateRandomProblem({
    randomSeed: 1,
    numWaypointPairs: 5,
    numObstacles: 3,
    minSpacing: 5,
  })

  const solver = new AngledTraceSolver(problem as CurvyTraceProblem)
  solver.solve()

  const outputTraces = solver.outputTraces
  expect(outputTraces.length).toBe(5)

  const invalidSegments: Array<{
    traceIndex: number
    segmentIndex: number
    angle: number
    dx: number
    dy: number
    p1: { x: number; y: number }
    p2: { x: number; y: number }
  }> = []

  for (let traceIdx = 0; traceIdx < outputTraces.length; traceIdx++) {
    const trace = outputTraces[traceIdx]
    const points = trace.points

    for (let i = 0; i < points.length - 1; i++) {
      const p1 = points[i]
      const p2 = points[i + 1]

      // Skip very short segments (could be duplicates or numerical noise)
      if (Math.hypot(p2.x - p1.x, p2.y - p1.y) < 1e-6) continue

      const result = getSegmentAngle(p1.x, p1.y, p2.x, p2.y)

      if (!result.valid) {
        invalidSegments.push({
          traceIndex: traceIdx,
          segmentIndex: i,
          angle: result.angle,
          dx: result.dx,
          dy: result.dy,
          p1,
          p2,
        })
      }
    }
  }

  if (invalidSegments.length > 0) {
    console.log("Invalid segments found:")
    for (const seg of invalidSegments) {
      console.log(
        `  Trace ${seg.traceIndex}, Segment ${seg.segmentIndex}: ` +
          `angle=${seg.angle.toFixed(2)}°, dx=${seg.dx.toFixed(4)}, dy=${seg.dy.toFixed(4)}, ` +
          `p1=(${seg.p1.x.toFixed(2)}, ${seg.p1.y.toFixed(2)}), p2=(${seg.p2.x.toFixed(2)}, ${seg.p2.y.toFixed(2)})`,
      )
    }
  }

  expect(invalidSegments).toEqual([])
})
