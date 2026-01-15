import { BaseSolver } from "@tscircuit/solver-utils"
import type { CurvyTraceProblem, OutputTrace, WaypointPair } from "./types"
import type { GraphicsObject } from "graphics-debug"
import type { Point, Bounds } from "@tscircuit/math-utils"
import { visualizeCurvyTraceProblem } from "./visualization-utils"
import { getObstacleOuterSegments } from "./geometry/getObstacleOuterSegments"

/**
 * AngledTraceSolver creates traces with 45-degree angle bends.
 * Each trace has 3 control parameters: [d1, d2, bendSign]
 * - d1: distance along perpendicular direction from start before first turn
 * - d2: distance along perpendicular direction from end before second turn
 * - bendSign: controls which direction to bend (-1 to +1, continuous for optimization)
 */

interface AngledTraceWithControlPoints {
  waypointPair: WaypointPair
  networkId?: string
  t1: number
  t2: number
  // Perpendicular directions at start and end
  perpDir1: Point
  perpDir2: Point
  // Control parameters: [d1, d2, bendSign]
  controlParams: [number, number, number]
  // Computed points for the trace (4 or 5 points depending on geometry)
  points: Point[]
  containedBy: number[]
  contains: number[]
}

// Get perimeter position for a point on the boundary
function getPerimeterT(p: Point, bounds: Bounds): number {
  const { minX, maxX, minY, maxY } = bounds
  const W = maxX - minX
  const H = maxY - minY
  const eps = 1e-6

  if (Math.abs(p.y - maxY) < eps) return p.x - minX
  if (Math.abs(p.x - maxX) < eps) return W + (maxY - p.y)
  if (Math.abs(p.y - minY) < eps) return W + H + (maxX - p.x)
  if (Math.abs(p.x - minX) < eps) return 2 * W + H + (p.y - minY)
  return 0
}

// Get the inward perpendicular direction for a point on the boundary
function getInwardPerpendicular(p: Point, bounds: Bounds): Point {
  const { minX, maxX, minY, maxY } = bounds
  const eps = 1e-6

  const onTop = Math.abs(p.y - maxY) < eps
  const onBottom = Math.abs(p.y - minY) < eps
  const onRight = Math.abs(p.x - maxX) < eps
  const onLeft = Math.abs(p.x - minX) < eps

  // Handle corners - perpendicular points diagonally inward
  if (onTop && onRight) return { x: -Math.SQRT1_2, y: -Math.SQRT1_2 }
  if (onTop && onLeft) return { x: Math.SQRT1_2, y: -Math.SQRT1_2 }
  if (onBottom && onRight) return { x: -Math.SQRT1_2, y: Math.SQRT1_2 }
  if (onBottom && onLeft) return { x: Math.SQRT1_2, y: Math.SQRT1_2 }

  // Handle edges
  if (onTop) return { x: 0, y: -1 }
  if (onBottom) return { x: 0, y: 1 }
  if (onRight) return { x: -1, y: 0 }
  if (onLeft) return { x: 1, y: 0 }

  // Fallback: point toward center
  const cx = (minX + maxX) / 2
  const cy = (minY + maxY) / 2
  const dx = cx - p.x
  const dy = cy - p.y
  const len = Math.hypot(dx, dy)
  return len > 0 ? { x: dx / len, y: dy / len } : { x: 0, y: -1 }
}

// Check if chord A contains chord B
function chordContains(
  aT1: number,
  aT2: number,
  bT1: number,
  bT2: number,
  perimeter: number,
): boolean {
  const normalize = (t: number) => ((t % perimeter) + perimeter) % perimeter
  const a1 = normalize(aT1),
    a2 = normalize(aT2)
  const b1 = normalize(bT1),
    b2 = normalize(bT2)
  const [aMin, aMax] = a1 < a2 ? [a1, a2] : [a2, a1]
  return b1 > aMin && b1 < aMax && b2 > aMin && b2 < aMax
}

function getBoundsCenter(bounds: Bounds): Point {
  return {
    x: (bounds.minX + bounds.maxX) / 2,
    y: (bounds.minY + bounds.maxY) / 2,
  }
}

// Inlined segment-to-segment distance squared for performance
function segmentDistSq(
  a1x: number,
  a1y: number,
  a2x: number,
  a2y: number,
  b1x: number,
  b1y: number,
  b2x: number,
  b2y: number,
): number {
  // Check for intersection first using cross products
  const bDx = b2x - b1x
  const bDy = b2y - b1y
  const aDx = a2x - a1x
  const aDy = a2y - a1y

  const d1 = bDx * (a1y - b1y) - bDy * (a1x - b1x)
  const d2 = bDx * (a2y - b1y) - bDy * (a2x - b1x)

  if (d1 > 0 !== d2 > 0) {
    const d3 = aDx * (b1y - a1y) - aDy * (b1x - a1x)
    const d4 = aDx * (b2y - a1y) - aDy * (b2x - a1x)
    if (d3 > 0 !== d4 > 0) {
      return 0 // Segments intersect
    }
  }

  // Compute 4 point-to-segment distances inline
  const bLenSq = bDx * bDx + bDy * bDy
  const aLenSq = aDx * aDx + aDy * aDy

  let minDistSq = Infinity
  let t: number
  let projX: number
  let projY: number
  let dpx: number
  let dpy: number
  let distSq: number

  // a1 to segment b
  if (bLenSq > 0) {
    t = ((a1x - b1x) * bDx + (a1y - b1y) * bDy) / bLenSq
    t = t < 0 ? 0 : t > 1 ? 1 : t
    projX = b1x + t * bDx
    projY = b1y + t * bDy
  } else {
    projX = b1x
    projY = b1y
  }
  dpx = a1x - projX
  dpy = a1y - projY
  distSq = dpx * dpx + dpy * dpy
  if (distSq < minDistSq) minDistSq = distSq

  // a2 to segment b
  if (bLenSq > 0) {
    t = ((a2x - b1x) * bDx + (a2y - b1y) * bDy) / bLenSq
    t = t < 0 ? 0 : t > 1 ? 1 : t
    projX = b1x + t * bDx
    projY = b1y + t * bDy
  } else {
    projX = b1x
    projY = b1y
  }
  dpx = a2x - projX
  dpy = a2y - projY
  distSq = dpx * dpx + dpy * dpy
  if (distSq < minDistSq) minDistSq = distSq

  // b1 to segment a
  if (aLenSq > 0) {
    t = ((b1x - a1x) * aDx + (b1y - a1y) * aDy) / aLenSq
    t = t < 0 ? 0 : t > 1 ? 1 : t
    projX = a1x + t * aDx
    projY = a1y + t * aDy
  } else {
    projX = a1x
    projY = a1y
  }
  dpx = b1x - projX
  dpy = b1y - projY
  distSq = dpx * dpx + dpy * dpy
  if (distSq < minDistSq) minDistSq = distSq

  // b2 to segment a
  if (aLenSq > 0) {
    t = ((b2x - a1x) * aDx + (b2y - a1y) * aDy) / aLenSq
    t = t < 0 ? 0 : t > 1 ? 1 : t
    projX = a1x + t * aDx
    projY = a1y + t * aDy
  } else {
    projX = a1x
    projY = a1y
  }
  dpx = b2x - projX
  dpy = b2y - projY
  distSq = dpx * dpx + dpy * dpy
  if (distSq < minDistSq) minDistSq = distSq

  return minDistSq
}

// Check if two segments intersect
function segmentsIntersect(
  a1x: number,
  a1y: number,
  a2x: number,
  a2y: number,
  b1x: number,
  b1y: number,
  b2x: number,
  b2y: number,
): boolean {
  const d1 = (b2x - b1x) * (a1y - b1y) - (b2y - b1y) * (a1x - b1x)
  const d2 = (b2x - b1x) * (a2y - b1y) - (b2y - b1y) * (a2x - b1x)
  const d3 = (a2x - a1x) * (b1y - a1y) - (a2y - a1y) * (b1x - a1x)
  const d4 = (a2x - a1x) * (b2y - a1y) - (a2y - a1y) * (b2x - a1x)

  return (
    ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
    ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
  )
}

// Compute bounding box for trace points
function computeTraceBounds(points: Point[]): {
  minX: number
  maxX: number
  minY: number
  maxY: number
} {
  let minX = Infinity,
    maxX = -Infinity,
    minY = Infinity,
    maxY = -Infinity
  for (const p of points) {
    if (p.x < minX) minX = p.x
    if (p.x > maxX) maxX = p.x
    if (p.y < minY) minY = p.y
    if (p.y > maxY) maxY = p.y
  }
  return { minX, maxX, minY, maxY }
}

/**
 * Check if a segment is at a valid angle (horizontal, vertical, or 45°)
 */
function isValidAngle(p1: Point, p2: Point): boolean {
  const dx = Math.abs(p2.x - p1.x)
  const dy = Math.abs(p2.y - p1.y)
  const eps = 1e-6

  // Horizontal: dy ≈ 0
  if (dy < eps) return true
  // Vertical: dx ≈ 0
  if (dx < eps) return true
  // 45°: |dx| ≈ |dy|
  if (Math.abs(dx - dy) < eps) return true

  return false
}

/**
 * Snap a point so that the segment from prev to this point is at a valid 45° angle.
 * Keeps the general direction but adjusts to nearest valid angle.
 */
function snapToValid45(prev: Point, target: Point): Point {
  const dx = target.x - prev.x
  const dy = target.y - prev.y

  const absDx = Math.abs(dx)
  const absDy = Math.abs(dy)

  // If already valid, return as-is
  if (absDy < 1e-6 || absDx < 1e-6 || Math.abs(absDx - absDy) < 1e-6) {
    return target
  }

  // Snap to nearest valid angle
  // Option 1: Make it horizontal (keep x, snap y)
  // Option 2: Make it vertical (keep y, snap x)
  // Option 3: Make it 45° (adjust to equal dx/dy)

  // Choose the option that's closest to the original target
  const horizontal: Point = { x: target.x, y: prev.y }
  const vertical: Point = { x: prev.x, y: target.y }

  // For 45°, we can either extend dx to match dy, or extend dy to match dx
  const diag45_extendX: Point = {
    x: prev.x + Math.sign(dx) * absDy,
    y: target.y,
  }
  const diag45_extendY: Point = {
    x: target.x,
    y: prev.y + Math.sign(dy) * absDx,
  }

  // Find which is closest to target
  const options = [horizontal, vertical, diag45_extendX, diag45_extendY]
  let best = target
  let bestDist = Infinity

  for (const opt of options) {
    const dist = Math.hypot(opt.x - target.x, opt.y - target.y)
    if (dist < bestDist) {
      bestDist = dist
      best = opt
    }
  }

  return best
}

/**
 * Compute dot product of two direction vectors
 */
function getDotProduct(
  dir1x: number,
  dir1y: number,
  dir2x: number,
  dir2y: number,
): number {
  return dir1x * dir2x + dir1y * dir2y
}

/**
 * Compute a direct 45-degree route from start to end.
 * This route stays as close as possible to the straight line path,
 * decomposing into valid 45° segments with minimal deviation.
 *
 * @param diagonalFirst - if true, do diagonal segment first; if false, do orthogonal first
 */
function computeDirectRoute(start: Point, end: Point, diagonalFirst: boolean = true): Point[] {
  const eps = 1e-6
  const dx = end.x - start.x
  const dy = end.y - start.y

  const absDx = Math.abs(dx)
  const absDy = Math.abs(dy)

  // If already at a valid angle, go direct
  if (absDx < eps || absDy < eps || Math.abs(absDx - absDy) < eps) {
    return [start, end]
  }

  // Decompose into diagonal + orthogonal
  const diagDist = Math.min(absDx, absDy)
  const diagX = Math.sign(dx) * diagDist
  const diagY = Math.sign(dy) * diagDist

  // The remaining orthogonal component
  const orthoX = dx - diagX
  const orthoY = dy - diagY

  if (diagonalFirst) {
    // Create midpoint - diagonal first
    const mid: Point = {
      x: start.x + diagX,
      y: start.y + diagY,
    }
    return [start, mid, end]
  } else {
    // Orthogonal first
    const mid: Point = {
      x: start.x + orthoX,
      y: start.y + orthoY,
    }
    return [start, mid, end]
  }
}

/**
 * Compute the 45-degree angled trace points from control parameters.
 * All segments are strictly horizontal, vertical, or 45-degree diagonal.
 *
 * The trace structure is:
 * start -> turn1 -> [mid] -> turn2 -> end
 *
 * Where:
 * - start to turn1: perpendicular to start boundary (horizontal or vertical)
 * - turn1 to turn2: decomposed into 45° diagonal + orthogonal
 * - turn2 to end: perpendicular to end boundary (horizontal or vertical)
 */
function computeAngledTracePoints(
  start: Point,
  end: Point,
  perpDir1: Point,
  perpDir2: Point,
  d1: number,
  d2: number,
  bendSign: number,
  bounds: Bounds,
): Point[] {
  const { minX, maxX, minY, maxY } = bounds
  const eps = 1e-6

  // If d1 and d2 are both very small, use direct routing
  // bendSign controls diagonal-first vs orthogonal-first
  if (d1 < eps && d2 < eps) {
    return computeDirectRoute(start, end, bendSign >= 0)
  }

  // For 45° routing, perpendicular directions must be axis-aligned (horizontal or vertical)
  // Corners would give diagonal perpendiculars, which we snap to the nearest axis
  const snapToOrthogonal = (dir: Point): Point => {
    // Snap to nearest horizontal or vertical direction
    if (Math.abs(dir.x) >= Math.abs(dir.y)) {
      return { x: Math.sign(dir.x) || 1, y: 0 }
    } else {
      return { x: 0, y: Math.sign(dir.y) || 1 }
    }
  }

  // Use axis-aligned perpendicular directions
  const orthoDir1 = snapToOrthogonal(perpDir1)
  const orthoDir2 = snapToOrthogonal(perpDir2)

  // Constrain d1 and d2 to stay within bounds
  const maxD1 =
    orthoDir1.x !== 0
      ? orthoDir1.x > 0
        ? maxX - start.x
        : start.x - minX
      : orthoDir1.y > 0
        ? maxY - start.y
        : start.y - minY
  const maxD2 =
    orthoDir2.x !== 0
      ? orthoDir2.x > 0
        ? maxX - end.x
        : end.x - minX
      : orthoDir2.y > 0
        ? maxY - end.y
        : end.y - minY

  const safeD1 = Math.min(d1, Math.max(0, maxD1 - 1))
  const safeD2 = Math.min(d2, Math.max(0, maxD2 - 1))

  // Turn1: go perpendicular from start
  const turn1: Point = {
    x: start.x + safeD1 * orthoDir1.x,
    y: start.y + safeD1 * orthoDir1.y,
  }

  // Turn2: go perpendicular from end
  const turn2: Point = {
    x: end.x + safeD2 * orthoDir2.x,
    y: end.y + safeD2 * orthoDir2.y,
  }

  // Vector from turn1 to turn2
  const dx = turn2.x - turn1.x
  const dy = turn2.y - turn1.y

  // If turn1 and turn2 are the same, just return simple path
  if (Math.abs(dx) < eps && Math.abs(dy) < eps) {
    return [start, turn1, end]
  }

  // Decompose into diagonal (45°) and orthogonal components
  const absDx = Math.abs(dx)
  const absDy = Math.abs(dy)

  // Diagonal travels equal distance in x and y
  const diagDist = Math.min(absDx, absDy)
  const diagX = Math.sign(dx) * diagDist
  const diagY = Math.sign(dy) * diagDist

  // Orthogonal is whatever remains (purely horizontal or purely vertical)
  const orthoX = dx - diagX
  const orthoY = dy - diagY

  // Determine the direction we need to go from turn1 to turn2
  const hasDiagonal = diagDist > eps
  const hasOrthogonal = Math.abs(orthoX) > eps || Math.abs(orthoY) > eps

  // Check if starting with diagonal or orthogonal would create a 90° turn
  // orthoDir1 is the direction of segment start->turn1
  // We want to avoid 90° turns at turn1

  let diagonalFirst = true // Default to diagonal first
  if (hasDiagonal && hasOrthogonal) {
    const normalizedBend = Math.tanh(bendSign)

    // Calculate dot products to see which order creates better angles
    const diagDirX = diagX / diagDist
    const diagDirY = diagY / diagDist

    const orthoLen = Math.hypot(orthoX, orthoY)
    const orthoDirX = orthoX / orthoLen
    const orthoDirY = orthoY / orthoLen

    // Dot with orthoDir1 (direction into turn1)
    const dotDiag1 = getDotProduct(orthoDir1.x, orthoDir1.y, diagDirX, diagDirY)
    const dotOrtho1 = getDotProduct(
      orthoDir1.x,
      orthoDir1.y,
      orthoDirX,
      orthoDirY,
    )

    // Dot with -orthoDir2 (direction into turn2)
    const dotDiag2 = getDotProduct(
      -orthoDir2.x,
      -orthoDir2.y,
      diagDirX,
      diagDirY,
    )
    const dotOrtho2 = getDotProduct(
      -orthoDir2.x,
      -orthoDir2.y,
      orthoDirX,
      orthoDirY,
    )

    // Score each option (higher is better - means smaller turns)
    // Diagonal first: turn1 angle uses diagonal, turn2 angle uses orthogonal
    const scoreDiagFirst = Math.min(dotDiag1, dotOrtho2)
    // Orthogonal first: turn1 angle uses orthogonal, turn2 angle uses diagonal
    const scoreOrthoFirst = Math.min(dotOrtho1, dotDiag2)

    // Prefer the option with better (larger) minimum angle
    // But also factor in bendSign preference
    if (scoreOrthoFirst > scoreDiagFirst + 0.3) {
      diagonalFirst = false
    } else if (scoreDiagFirst > scoreOrthoFirst + 0.3) {
      diagonalFirst = true
    } else {
      // Scores are similar, use bendSign
      diagonalFirst = normalizedBend >= 0
    }
  }

  // Build the path
  const points: Point[] = [start, turn1]

  if (hasDiagonal && hasOrthogonal) {
    if (diagonalFirst) {
      // Diagonal first, then orthogonal
      const mid: Point = {
        x: turn1.x + diagX,
        y: turn1.y + diagY,
      }
      points.push(mid)
    } else {
      // Orthogonal first, then diagonal
      const mid: Point = {
        x: turn1.x + orthoX,
        y: turn1.y + orthoY,
      }
      points.push(mid)
    }
  }
  // If only diagonal or only orthogonal, no intermediate point needed

  points.push(turn2, end)

  // Remove duplicate consecutive points
  let filtered: Point[] = [points[0]]
  for (let i = 1; i < points.length; i++) {
    const prev = filtered[filtered.length - 1]
    const curr = points[i]
    if (Math.hypot(curr.x - prev.x, curr.y - prev.y) > eps) {
      filtered.push(curr)
    }
  }

  // Chamfer sharp corners (90° or sharper) by inserting diagonal segments
  filtered = chamferSharpCorners(filtered)

  return filtered
}

/**
 * Insert chamfer points at sharp corners to convert 90° turns into two 45° turns.
 * For a corner A -> B -> C with a 90° turn at B, we replace it with:
 * A -> B1 -> B2 -> C where B1 and B2 create two 45° turns.
 */
function chamferSharpCorners(points: Point[]): Point[] {
  if (points.length < 3) return points

  const eps = 1e-6
  const result: Point[] = [points[0]]

  for (let i = 1; i < points.length - 1; i++) {
    const before = points[i - 1]
    const corner = points[i]
    const after = points[i + 1]

    // Calculate directions
    const dx1 = corner.x - before.x
    const dy1 = corner.y - before.y
    const len1 = Math.hypot(dx1, dy1)

    const dx2 = after.x - corner.x
    const dy2 = after.y - corner.y
    const len2 = Math.hypot(dx2, dy2)

    if (len1 < eps || len2 < eps) {
      result.push(corner)
      continue
    }

    // Normalized directions
    const dir1x = dx1 / len1
    const dir1y = dy1 / len1
    const dir2x = dx2 / len2
    const dir2y = dy2 / len2

    // Check if this is a sharp turn (dot < 0.5 means >= 60° turn)
    const dot = getDotProduct(dir1x, dir1y, dir2x, dir2y)

    if (dot >= 0.5) {
      // Not a sharp turn, keep the corner as-is
      result.push(corner)
      continue
    }

    // Sharp turn detected - insert a diagonal chamfer
    // The chamfer distance should be proportional to the shorter segment
    const chamferDist = Math.min(len1, len2) * 0.4

    // Create chamfer points:
    // B1 is before the corner (along the incoming segment)
    // B2 is after the corner (along the outgoing segment)
    const b1: Point = {
      x: corner.x - dir1x * chamferDist,
      y: corner.y - dir1y * chamferDist,
    }
    const b2: Point = {
      x: corner.x + dir2x * chamferDist,
      y: corner.y + dir2y * chamferDist,
    }

    // Add chamfer points
    result.push(b1, b2)
  }

  result.push(points[points.length - 1])
  return result
}

/**
 * Sample points along a polyline at regular intervals
 */
function samplePolyline(points: Point[], numSamples: number): Point[] {
  if (points.length < 2) return [...points]

  // Compute total length
  let totalLength = 0
  const segmentLengths: number[] = []
  for (let i = 0; i < points.length - 1; i++) {
    const len = Math.hypot(
      points[i + 1].x - points[i].x,
      points[i + 1].y - points[i].y,
    )
    segmentLengths.push(len)
    totalLength += len
  }

  if (totalLength < 1e-9) return [points[0], points[points.length - 1]]

  const sampled: Point[] = []
  for (let s = 0; s <= numSamples; s++) {
    const targetDist = (s / numSamples) * totalLength
    let accumulated = 0
    let found = false

    for (let i = 0; i < segmentLengths.length; i++) {
      if (accumulated + segmentLengths[i] >= targetDist - 1e-9) {
        const segDist = targetDist - accumulated
        const t = segmentLengths[i] > 1e-9 ? segDist / segmentLengths[i] : 0
        sampled.push({
          x: points[i].x + t * (points[i + 1].x - points[i].x),
          y: points[i].y + t * (points[i + 1].y - points[i].y),
        })
        found = true
        break
      }
      accumulated += segmentLengths[i]
    }

    if (!found) {
      sampled.push(points[points.length - 1])
    }
  }

  return sampled
}

const OPT_SAMPLES = 6 // Balanced for speed and collision detection
const OUTPUT_SAMPLES = 20

export class AngledTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traces: AngledTraceWithControlPoints[] = []
  private optimizationStep = 0
  private readonly maxOptimizationSteps = 60 // Reduced for speed - early termination handles convergence

  // Sampled points for cost computation
  private sampledPoints: Float64Array[] = []
  private traceBounds: {
    minX: number
    maxX: number
    minY: number
    maxY: number
  }[] = []

  // Pre-computed obstacle segments
  private obstacleSegments: Float64Array = new Float64Array(0)
  private obstacleNetworkIds: (string | undefined)[] = []
  private numObstacleSegments = 0

  // Collision pairs cache
  private collisionPairs: [number, number][] = []

  private lastCost = Infinity
  private stagnantSteps = 0

  // Effective spacing that decays during optimization
  private effectiveTraceToTraceSpacing: number = 0

  constructor(public problem: CurvyTraceProblem) {
    super()
    for (const obstacle of this.problem.obstacles) {
      obstacle.outerSegments = getObstacleOuterSegments(obstacle)
    }
    this.precomputeObstacles()
  }

  override getConstructorParams() {
    return this.problem
  }

  /**
   * Get the current solution weights as Array<[number, number, number]>
   */
  getSolutionWeights(): Array<[number, number, number]> {
    return this.traces.map(
      (t) => [...t.controlParams] as [number, number, number],
    )
  }

  /**
   * Set the solution weights
   */
  setSolutionWeights(weights: Array<[number, number, number]>) {
    if (this.traces.length === 0) {
      this.initializeTraces()
    }
    for (let i = 0; i < Math.min(weights.length, this.traces.length); i++) {
      this.traces[i].controlParams = [...weights[i]]
      this.updateTracePoints(i)
    }
    this.updateAllSampledTraces()
    this.updateCollisionPairs()
  }

  private precomputeObstacles() {
    const { obstacles } = this.problem
    let totalSegments = 0
    for (const obs of obstacles) {
      if (obs.outerSegments) totalSegments += obs.outerSegments.length
    }

    this.obstacleSegments = new Float64Array(totalSegments * 4)
    this.obstacleNetworkIds = []
    this.numObstacleSegments = totalSegments

    let idx = 0
    for (const obs of obstacles) {
      if (obs.outerSegments) {
        for (const seg of obs.outerSegments) {
          this.obstacleSegments[idx++] = seg[0].x
          this.obstacleSegments[idx++] = seg[0].y
          this.obstacleSegments[idx++] = seg[1].x
          this.obstacleSegments[idx++] = seg[1].y
          this.obstacleNetworkIds.push(obs.networkId)
        }
      }
    }
  }

  private initializeTraces() {
    const { bounds, waypointPairs } = this.problem
    const { minX, maxX, minY, maxY } = bounds
    const W = maxX - minX
    const H = maxY - minY
    const perimeter = 2 * W + 2 * H
    const center = getBoundsCenter(bounds)

    const tracesWithT = waypointPairs.map((pair, idx) => ({
      pair,
      t1: getPerimeterT(pair.start, bounds),
      t2: getPerimeterT(pair.end, bounds),
      idx,
    }))

    // Compute containment relationships
    const containedBy: number[][] = tracesWithT.map(() => [])
    const contains: number[][] = tracesWithT.map(() => [])

    for (const trace of tracesWithT) {
      for (const other of tracesWithT) {
        if (trace.idx !== other.idx) {
          if (
            trace.pair.networkId &&
            other.pair.networkId &&
            trace.pair.networkId === other.pair.networkId
          ) {
            continue
          }
          if (
            chordContains(other.t1, other.t2, trace.t1, trace.t2, perimeter)
          ) {
            containedBy[trace.idx].push(other.idx)
            contains[other.idx].push(trace.idx)
          }
        }
      }
    }

    // Compute nesting depth
    const nestingDepth = containedBy.map((arr) => arr.length)
    const maxDepth = Math.max(...nestingDepth, 1)

    // Initialize all traces with direct routing (d1=0, d2=0)
    // This keeps traces as close as possible to their straight-line paths
    this.traces = tracesWithT.map(({ pair, t1, t2, idx }) => {
      const perpDir1 = getInwardPerpendicular(pair.start, bounds)
      const perpDir2 = getInwardPerpendicular(pair.end, bounds)

      // Start with direct route (no perpendicular offsets)
      const d1 = 0
      const d2 = 0
      const bendSign = 0

      // Compute initial points using direct route
      const points = computeAngledTracePoints(
        pair.start,
        pair.end,
        perpDir1,
        perpDir2,
        d1,
        d2,
        bendSign,
        bounds,
      )

      return {
        waypointPair: pair,
        networkId: pair.networkId,
        t1,
        t2,
        perpDir1,
        perpDir2,
        controlParams: [d1, d2, bendSign] as [number, number, number],
        points,
        containedBy: containedBy[idx],
        contains: contains[idx],
      }
    })

    // Initialize sampled points arrays
    this.sampledPoints = this.traces.map(
      () => new Float64Array((OPT_SAMPLES + 1) * 2),
    )
    this.traceBounds = this.traces.map(() => ({
      minX: 0,
      maxX: 0,
      minY: 0,
      maxY: 0,
    }))

    this.updateAllSampledTraces()
    this.updateCollisionPairs()
  }

  private updateTracePoints(i: number) {
    const trace = this.traces[i]
    const [d1, d2, bendSign] = trace.controlParams
    trace.points = computeAngledTracePoints(
      trace.waypointPair.start,
      trace.waypointPair.end,
      trace.perpDir1,
      trace.perpDir2,
      d1,
      d2,
      bendSign,
      this.problem.bounds,
    )
  }

  private updateAllSampledTraces() {
    for (let i = 0; i < this.traces.length; i++) {
      this.updateSingleTraceSample(i)
    }
  }

  private updateSingleTraceSample(i: number) {
    const trace = this.traces[i]
    const sampled = samplePolyline(trace.points, OPT_SAMPLES)

    // Copy to typed array
    for (let j = 0; j < sampled.length; j++) {
      this.sampledPoints[i][j * 2] = sampled[j].x
      this.sampledPoints[i][j * 2 + 1] = sampled[j].y
    }

    this.traceBounds[i] = computeTraceBounds(trace.points)
  }

  private updateCollisionPairs() {
    const effectiveSpacing = this.effectiveTraceToTraceSpacing
    this.collisionPairs = []

    for (let i = 0; i < this.traces.length; i++) {
      for (let j = i + 1; j < this.traces.length; j++) {
        const ti = this.traces[i],
          tj = this.traces[j]
        if (ti.networkId && tj.networkId && ti.networkId === tj.networkId)
          continue

        const bi = this.traceBounds[i],
          bj = this.traceBounds[j]
        if (
          bi.maxX + effectiveSpacing >= bj.minX &&
          bj.maxX + effectiveSpacing >= bi.minX &&
          bi.maxY + effectiveSpacing >= bj.minY &&
          bj.maxY + effectiveSpacing >= bi.minY
        ) {
          this.collisionPairs.push([i, j])
        }
      }
    }
  }

  private computeTotalCost(): number {
    const { preferredObstacleToTraceSpacing } = this.problem
    const effectiveSpacing = this.effectiveTraceToTraceSpacing
    const traceSpacingSq = effectiveSpacing ** 2
    const obstacleSpacingSq = preferredObstacleToTraceSpacing ** 2
    let cost = 0

    // Cost between traces
    for (const [i, j] of this.collisionPairs) {
      const pi = this.sampledPoints[i],
        pj = this.sampledPoints[j]

      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2],
          a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2],
          a2y = pi[(a + 1) * 2 + 1]

        for (let b = 0; b < OPT_SAMPLES; b++) {
          const b1x = pj[b * 2],
            b1y = pj[b * 2 + 1]
          const b2x = pj[(b + 1) * 2],
            b2y = pj[(b + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
          if (distSq < traceSpacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (effectiveSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * traceSpacingSq
          }
        }
      }
    }

    // Cost against obstacles
    for (let i = 0; i < this.traces.length; i++) {
      const trace = this.traces[i]
      const pi = this.sampledPoints[i]
      const bi = this.traceBounds[i]

      for (let obsIdx = 0; obsIdx < this.numObstacleSegments; obsIdx++) {
        if (
          trace.networkId &&
          this.obstacleNetworkIds[obsIdx] &&
          trace.networkId === this.obstacleNetworkIds[obsIdx]
        )
          continue

        const obsBase = obsIdx * 4
        const ox1 = this.obstacleSegments[obsBase]
        const oy1 = this.obstacleSegments[obsBase + 1]
        const ox2 = this.obstacleSegments[obsBase + 2]
        const oy2 = this.obstacleSegments[obsBase + 3]

        const obsMinX = Math.min(ox1, ox2),
          obsMaxX = Math.max(ox1, ox2)
        const obsMinY = Math.min(oy1, oy2),
          obsMaxY = Math.max(oy1, oy2)
        if (
          bi.maxX + preferredObstacleToTraceSpacing < obsMinX ||
          obsMaxX + preferredObstacleToTraceSpacing < bi.minX ||
          bi.maxY + preferredObstacleToTraceSpacing < obsMinY ||
          obsMaxY + preferredObstacleToTraceSpacing < bi.minY
        )
          continue

        for (let a = 0; a < OPT_SAMPLES; a++) {
          const a1x = pi[a * 2],
            a1y = pi[a * 2 + 1]
          const a2x = pi[(a + 1) * 2],
            a2y = pi[(a + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, ox1, oy1, ox2, oy2)
          if (distSq < obstacleSpacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (preferredObstacleToTraceSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * obstacleSpacingSq
          }
        }
      }
    }

    // Add shape penalties for all traces
    for (let i = 0; i < this.traces.length; i++) {
      cost += this.computeShapePenalty(i)
    }

    return cost
  }

  // Fast cost using actual trace segments (not samples)
  private computeCostForTraceFast(traceIdx: number): number {
    const { preferredObstacleToTraceSpacing } = this.problem
    const effectiveSpacing = this.effectiveTraceToTraceSpacing
    const traceSpacingSq = effectiveSpacing ** 2
    const obstacleSpacingSq = preferredObstacleToTraceSpacing ** 2
    const trace = this.traces[traceIdx]
    const pts = trace.points
    const bi = this.traceBounds[traceIdx]
    let cost = 0

    // Very high penalty for actual intersections
    const INTERSECTION_PENALTY = 10000

    // Cost against other traces using actual segments
    for (let j = 0; j < this.traces.length; j++) {
      if (j === traceIdx) continue
      const other = this.traces[j]
      if (
        trace.networkId &&
        other.networkId &&
        trace.networkId === other.networkId
      )
        continue

      const bj = this.traceBounds[j]
      if (
        bi.maxX + effectiveSpacing < bj.minX ||
        bj.maxX + effectiveSpacing < bi.minX ||
        bi.maxY + effectiveSpacing < bj.minY ||
        bj.maxY + effectiveSpacing < bi.minY
      )
        continue

      const otherPts = other.points
      for (let a = 0; a < pts.length - 1; a++) {
        const a1x = pts[a].x
        const a1y = pts[a].y
        const a2x = pts[a + 1].x
        const a2y = pts[a + 1].y

        for (let b = 0; b < otherPts.length - 1; b++) {
          const b1x = otherPts[b].x
          const b1y = otherPts[b].y
          const b2x = otherPts[b + 1].x
          const b2y = otherPts[b + 1].y

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
          if (distSq === 0) {
            // Actual intersection - very high penalty
            cost += INTERSECTION_PENALTY
          } else if (distSq < traceSpacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (effectiveSpacing - dist) ** 2
          }
        }
      }
    }

    // Cost against obstacles using actual segments
    for (let obsIdx = 0; obsIdx < this.numObstacleSegments; obsIdx++) {
      if (
        trace.networkId &&
        this.obstacleNetworkIds[obsIdx] &&
        trace.networkId === this.obstacleNetworkIds[obsIdx]
      )
        continue

      const obsBase = obsIdx * 4
      const ox1 = this.obstacleSegments[obsBase]
      const oy1 = this.obstacleSegments[obsBase + 1]
      const ox2 = this.obstacleSegments[obsBase + 2]
      const oy2 = this.obstacleSegments[obsBase + 3]

      for (let a = 0; a < pts.length - 1; a++) {
        const distSq = segmentDistSq(
          pts[a].x,
          pts[a].y,
          pts[a + 1].x,
          pts[a + 1].y,
          ox1,
          oy1,
          ox2,
          oy2,
        )
        if (distSq === 0) {
          cost += INTERSECTION_PENALTY
        } else if (distSq < obstacleSpacingSq) {
          const dist = Math.sqrt(distSq)
          cost += (preferredObstacleToTraceSpacing - dist) ** 2
        }
      }
    }

    return cost
  }

  private computeCostForTrace(traceIdx: number): number {
    const { preferredObstacleToTraceSpacing } = this.problem
    const effectiveSpacing = this.effectiveTraceToTraceSpacing
    const traceSpacingSq = effectiveSpacing ** 2
    const obstacleSpacingSq = preferredObstacleToTraceSpacing ** 2
    const trace = this.traces[traceIdx]
    const pi = this.sampledPoints[traceIdx]
    const bi = this.traceBounds[traceIdx]
    let cost = 0

    // Cost against other traces
    for (let j = 0; j < this.traces.length; j++) {
      if (j === traceIdx) continue
      const other = this.traces[j]
      if (
        trace.networkId &&
        other.networkId &&
        trace.networkId === other.networkId
      )
        continue

      const bj = this.traceBounds[j]
      if (
        bi.maxX + effectiveSpacing < bj.minX ||
        bj.maxX + effectiveSpacing < bi.minX ||
        bi.maxY + effectiveSpacing < bj.minY ||
        bj.maxY + effectiveSpacing < bi.minY
      )
        continue

      const pj = this.sampledPoints[j]
      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2]
        const a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2]
        const a2y = pi[(a + 1) * 2 + 1]

        for (let b = 0; b < OPT_SAMPLES; b++) {
          const b1x = pj[b * 2]
          const b1y = pj[b * 2 + 1]
          const b2x = pj[(b + 1) * 2]
          const b2y = pj[(b + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
          if (distSq < traceSpacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (effectiveSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * traceSpacingSq
          }
        }
      }
    }

    // Cost against obstacles
    for (let obsIdx = 0; obsIdx < this.numObstacleSegments; obsIdx++) {
      if (
        trace.networkId &&
        this.obstacleNetworkIds[obsIdx] &&
        trace.networkId === this.obstacleNetworkIds[obsIdx]
      )
        continue

      const obsBase = obsIdx * 4
      const ox1 = this.obstacleSegments[obsBase]
      const oy1 = this.obstacleSegments[obsBase + 1]
      const ox2 = this.obstacleSegments[obsBase + 2]
      const oy2 = this.obstacleSegments[obsBase + 3]

      const obsMinX = Math.min(ox1, ox2),
        obsMaxX = Math.max(ox1, ox2)
      const obsMinY = Math.min(oy1, oy2),
        obsMaxY = Math.max(oy1, oy2)
      if (
        bi.maxX + preferredObstacleToTraceSpacing < obsMinX ||
        obsMaxX + preferredObstacleToTraceSpacing < bi.minX ||
        bi.maxY + preferredObstacleToTraceSpacing < obsMinY ||
        obsMaxY + preferredObstacleToTraceSpacing < bi.minY
      )
        continue

      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2],
          a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2],
          a2y = pi[(a + 1) * 2 + 1]

        const distSq = segmentDistSq(a1x, a1y, a2x, a2y, ox1, oy1, ox2, oy2)
        if (distSq < obstacleSpacingSq) {
          const dist = Math.sqrt(distSq)
          cost += (preferredObstacleToTraceSpacing - dist) ** 2
          if (distSq < 1e-18) cost += 20 * obstacleSpacingSq
        }
      }
    }

    // Add shape penalty for this trace
    cost += this.computeShapePenalty(traceIdx)

    return cost
  }

  /**
   * Compute penalty for undesirable trace shapes:
   * - Sharp turns (90 degrees or more)
   * - 45 degree segments shorter than 5% of trace length
   *
   * For 45-degree routing, valid turn angles are:
   * - 0° (straight): dot product = 1
   * - 45° (to/from diagonal): dot product ≈ 0.707
   *
   * Invalid turns:
   * - 90° (perpendicular): dot product = 0
   * - 135° or sharper: dot product < 0
   */
  private computeShapePenalty(traceIdx: number): number {
    const trace = this.traces[traceIdx]
    const points = trace.points
    if (points.length < 2) return 0

    let penalty = 0
    const eps = 1e-6

    // Compute total trace length
    let totalLength = 0
    for (let i = 0; i < points.length - 1; i++) {
      totalLength += Math.hypot(
        points[i + 1].x - points[i].x,
        points[i + 1].y - points[i].y,
      )
    }

    if (totalLength < eps) return 0

    const minSegmentLength = totalLength * 0.05 // 5% of trace length

    // Check each segment and turn
    for (let i = 0; i < points.length - 1; i++) {
      const dx = points[i + 1].x - points[i].x
      const dy = points[i + 1].y - points[i].y
      const segLength = Math.hypot(dx, dy)

      if (segLength < eps) continue

      // Check if this is a 45 degree segment (|dx| ≈ |dy|)
      const absDx = Math.abs(dx)
      const absDy = Math.abs(dy)
      const is45Degree = Math.abs(absDx - absDy) < eps * segLength

      // Penalize short 45 degree segments
      if (is45Degree && segLength < minSegmentLength) {
        // Penalty proportional to how short the segment is
        const shortness = (minSegmentLength - segLength) / minSegmentLength
        penalty += shortness * shortness * 100
      }

      // Check for sharp turns (need at least 3 points)
      if (i < points.length - 2) {
        const dx2 = points[i + 2].x - points[i + 1].x
        const dy2 = points[i + 2].y - points[i + 1].y
        const seg2Length = Math.hypot(dx2, dy2)

        if (seg2Length > eps) {
          // Normalize direction vectors
          const dir1x = dx / segLength
          const dir1y = dy / segLength
          const dir2x = dx2 / seg2Length
          const dir2y = dy2 / seg2Length

          // Dot product: cos(angle between directions)
          // dot = 1: straight (0° turn)
          // dot ≈ 0.707: 45° turn (valid for 45° routing)
          // dot = 0: 90° turn (bad)
          // dot < 0: > 90° turn (very bad)
          const dot = dir1x * dir2x + dir1y * dir2y

          // For 45° routing, valid turns have dot ≈ 0.707 or dot = 1
          // Penalize any turn sharper than ~50° (dot < 0.6)
          if (dot < 0.6) {
            // 90° turn (dot ≈ 0): penalty = 100
            // 135° turn (dot ≈ -0.707): penalty = 170
            // 180° turn (dot = -1): penalty = 200
            const sharpness = 1 - dot // 0.4 for barely bad, 2.0 for U-turn
            penalty += sharpness * 100
          }
        }
      }
    }

    return penalty
  }

  private getTraceLength(i: number): number {
    const trace = this.traces[i]
    let length = 0
    for (let p = 0; p < trace.points.length - 1; p++) {
      length += Math.hypot(
        trace.points[p + 1].x - trace.points[p].x,
        trace.points[p + 1].y - trace.points[p].y,
      )
    }
    return length
  }

  private tracesIntersect(i: number, j: number): boolean {
    const trace1 = this.traces[i]
    const trace2 = this.traces[j]
    const p1 = trace1.points
    const p2 = trace2.points

    for (let a = 0; a < p1.length - 1; a++) {
      for (let b = 0; b < p2.length - 1; b++) {
        if (
          segmentsIntersect(
            p1[a].x,
            p1[a].y,
            p1[a + 1].x,
            p1[a + 1].y,
            p2[b].x,
            p2[b].y,
            p2[b + 1].x,
            p2[b + 1].y,
          )
        ) {
          return true
        }
      }
    }
    return false
  }

  private findIntersectingPairs(): [number, number][] {
    const intersecting: [number, number][] = []

    for (let i = 0; i < this.traces.length; i++) {
      for (let j = i + 1; j < this.traces.length; j++) {
        const ti = this.traces[i]
        const tj = this.traces[j]

        if (ti.networkId && tj.networkId && ti.networkId === tj.networkId)
          continue

        const bi = this.traceBounds[i]
        const bj = this.traceBounds[j]
        if (
          bi.maxX < bj.minX ||
          bj.maxX < bi.minX ||
          bi.maxY < bj.minY ||
          bj.maxY < bi.minY
        )
          continue

        if (this.tracesIntersect(i, j)) {
          intersecting.push([i, j])
        }
      }
    }

    return intersecting
  }

  private resolveIntersections(): number {
    const { bounds, preferredTraceToTraceSpacing } = this.problem
    const { minX, maxX, minY, maxY } = bounds
    const minDim = Math.min(maxX - minX, maxY - minY)
    const minDist = 0 // Allow zero for direct routing
    const maxDist = minDim * 2.0 // Allow larger distances for extreme cases

    const intersecting = this.findIntersectingPairs()
    if (intersecting.length === 0) return 0

    // Sort intersecting pairs: handle pairs with shorter combined trace length first
    // (shorter traces are easier to adjust)
    intersecting.sort((a, b) => {
      const lenA = this.getTraceLength(a[0]) + this.getTraceLength(a[1])
      const lenB = this.getTraceLength(b[0]) + this.getTraceLength(b[1])
      return lenA - lenB
    })

    let resolved = 0

    for (const [i, j] of intersecting) {
      const ti = this.traces[i]
      const tj = this.traces[j]

      if (!this.tracesIntersect(i, j)) continue

      // Determine outer vs inner trace based on containment
      let outerIdx: number
      let innerIdx: number

      if (ti.containedBy.includes(j)) {
        outerIdx = j
        innerIdx = i
      } else if (tj.containedBy.includes(i)) {
        outerIdx = i
        innerIdx = j
      } else {
        // Neither contains the other - use average distance as heuristic
        const avgDi = (ti.controlParams[0] + ti.controlParams[1]) / 2
        const avgDj = (tj.controlParams[0] + tj.controlParams[1]) / 2
        if (avgDi < avgDj) {
          outerIdx = i
          innerIdx = j
        } else {
          outerIdx = j
          innerIdx = i
        }
      }

      const outerTrace = this.traces[outerIdx]
      const innerTrace = this.traces[innerIdx]

      // Save original state
      const origOuter = [...outerTrace.controlParams] as [number, number, number]
      const origInner = [...innerTrace.controlParams] as [number, number, number]

      const separation = preferredTraceToTraceSpacing * 2

      // Balanced strategies - effective ones for intersection resolution
      type Strategy = {
        target: "inner" | "outer" | "both"
        innerD: number
        outerD: number
        innerBend: number
        outerBend: number
      }

      const strategies: Strategy[] = [
        // Distance adjustments (most common solutions)
        { target: "inner", innerD: separation, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 2, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 3, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "outer", innerD: 0, outerD: separation, innerBend: 0, outerBend: 0 },
        { target: "outer", innerD: 0, outerD: separation * 2, innerBend: 0, outerBend: 0 },
        // Bend changes
        { target: "inner", innerD: 0, outerD: 0, innerBend: -1, outerBend: 0 },
        { target: "inner", innerD: 0, outerD: 0, innerBend: 1, outerBend: 0 },
        { target: "outer", innerD: 0, outerD: 0, innerBend: 0, outerBend: -1 },
        { target: "outer", innerD: 0, outerD: 0, innerBend: 0, outerBend: 1 },
        // Medium distance
        { target: "inner", innerD: separation * 5, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 8, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "outer", innerD: 0, outerD: separation * 4, innerBend: 0, outerBend: 0 },
        // Combined distance + bend
        { target: "inner", innerD: separation * 2, outerD: 0, innerBend: -1, outerBend: 0 },
        { target: "inner", innerD: separation * 2, outerD: 0, innerBend: 1, outerBend: 0 },
        { target: "outer", innerD: 0, outerD: separation * 2, innerBend: 0, outerBend: -1 },
        { target: "outer", innerD: 0, outerD: separation * 2, innerBend: 0, outerBend: 1 },
        // Move both traces
        { target: "both", innerD: separation * 2, outerD: separation, innerBend: 0, outerBend: 0 },
        { target: "both", innerD: separation * 3, outerD: separation * 2, innerBend: 0, outerBend: 0 },
        { target: "both", innerD: 0, outerD: 0, innerBend: 1, outerBend: -1 },
        { target: "both", innerD: 0, outerD: 0, innerBend: -1, outerBend: 1 },
        // Large distance for stubborn cases
        { target: "inner", innerD: separation * 12, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 15, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 20, outerD: 0, innerBend: 0, outerBend: 0 },
        { target: "inner", innerD: separation * 25, outerD: 0, innerBend: 0, outerBend: 0 },
        // Combined large distance + bend
        { target: "inner", innerD: separation * 8, outerD: 0, innerBend: 2, outerBend: 0 },
        { target: "inner", innerD: separation * 8, outerD: 0, innerBend: -2, outerBend: 0 },
        { target: "inner", innerD: separation * 12, outerD: 0, innerBend: 1, outerBend: 0 },
        { target: "inner", innerD: separation * 12, outerD: 0, innerBend: -1, outerBend: 0 },
        // Push outer trace
        { target: "both", innerD: separation * 5, outerD: -separation * 2, innerBend: 0, outerBend: 0 },
        { target: "both", innerD: separation * 8, outerD: -separation * 3, innerBend: 0, outerBend: 0 },
      ]

      let foundSolution = false

      for (const strategy of strategies) {
        // Reset
        outerTrace.controlParams = [...origOuter]
        innerTrace.controlParams = [...origInner]

        // Apply strategy
        innerTrace.controlParams[0] = Math.min(maxDist, origInner[0] + strategy.innerD)
        innerTrace.controlParams[1] = Math.min(maxDist, origInner[1] + strategy.innerD)
        innerTrace.controlParams[2] = origInner[2] + strategy.innerBend

        outerTrace.controlParams[0] = Math.max(minDist, origOuter[0] - strategy.outerD)
        outerTrace.controlParams[1] = Math.max(minDist, origOuter[1] - strategy.outerD)
        outerTrace.controlParams[2] = origOuter[2] + strategy.outerBend

        this.updateTracePoints(innerIdx)
        this.updateTracePoints(outerIdx)
        this.updateSingleTraceSample(innerIdx)
        this.updateSingleTraceSample(outerIdx)

        if (!this.tracesIntersect(outerIdx, innerIdx)) {
          // Found a working solution - keep it
          resolved++
          foundSolution = true
          break
        }
      }

      if (!foundSolution) {
        // Try swapping roles with more strategies
        const swapStrategies = [
          { innerD: separation * 2, innerBend: 0 },
          { innerD: separation * 5, innerBend: 0 },
          { innerD: separation * 8, innerBend: 0 },
          { innerD: separation * 12, innerBend: 0 },
          { innerD: separation * 15, innerBend: 0 },
          { innerD: 0, innerBend: -1 },
          { innerD: 0, innerBend: 1 },
          { innerD: separation * 5, innerBend: -1 },
          { innerD: separation * 5, innerBend: 1 },
          { innerD: separation * 10, innerBend: -1 },
        ]
        for (const strategy of swapStrategies) {
          outerTrace.controlParams = [...origOuter]
          innerTrace.controlParams = [...origInner]

          outerTrace.controlParams[0] = Math.min(maxDist, origOuter[0] + strategy.innerD)
          outerTrace.controlParams[1] = Math.min(maxDist, origOuter[1] + strategy.innerD)
          outerTrace.controlParams[2] = origOuter[2] + strategy.innerBend

          this.updateTracePoints(innerIdx)
          this.updateTracePoints(outerIdx)
          this.updateSingleTraceSample(innerIdx)
          this.updateSingleTraceSample(outerIdx)

          if (!this.tracesIntersect(outerIdx, innerIdx)) {
            resolved++
            foundSolution = true
            break
          }
        }
      }

      if (!foundSolution) {
        // Revert to original if nothing worked
        outerTrace.controlParams = origOuter
        innerTrace.controlParams = origInner
        this.updateTracePoints(outerIdx)
        this.updateTracePoints(innerIdx)
        this.updateSingleTraceSample(outerIdx)
        this.updateSingleTraceSample(innerIdx)
      }
    }

    return resolved
  }

  private optimizeStep() {
    const { bounds } = this.problem
    const { minX, maxX, minY, maxY } = bounds
    const minDim = Math.min(maxX - minX, maxY - minY)

    const progress = this.optimizationStep / this.maxOptimizationSteps
    const step = 3.0 * (1 - progress) + 0.5

    // Allow d1/d2 to go to 0 for direct routing
    const minDist = 0
    const maxDist = minDim * 1.5

    // Process only traces with non-zero cost
    for (let i = 0; i < this.traces.length; i++) {
      const currentCost = this.computeCostForTrace(i)
      if (currentCost === 0) continue

      const trace = this.traces[i]
      const origParams: [number, number, number] = [
        trace.controlParams[0],
        trace.controlParams[1],
        trace.controlParams[2],
      ]
      let bestCost = currentCost
      let bestD1 = origParams[0]
      let bestD2 = origParams[1]
      let bestBend = origParams[2]

      const largeStep = step * 2
      const deltas = [step, -step, largeStep, -largeStep, step * 3, -step * 3]

      // Try d1 adjustments
      for (const delta of deltas) {
        const newD1 = origParams[0] + delta
        if (newD1 >= minDist && newD1 <= maxDist) {
          trace.controlParams[0] = newD1
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestD1 = newD1
          }
        }
      }
      trace.controlParams[0] = origParams[0]

      // Try d2 adjustments
      for (const delta of deltas) {
        const newD2 = origParams[1] + delta
        if (newD2 >= minDist && newD2 <= maxDist) {
          trace.controlParams[1] = newD2
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestD2 = newD2
            bestD1 = origParams[0]
          }
        }
      }
      trace.controlParams[1] = origParams[1]

      // Try both d1 and d2 together
      for (const delta of deltas) {
        const newD1 = origParams[0] + delta
        const newD2 = origParams[1] + delta
        if (
          newD1 >= minDist &&
          newD1 <= maxDist &&
          newD2 >= minDist &&
          newD2 <= maxDist
        ) {
          trace.controlParams[0] = newD1
          trace.controlParams[1] = newD2
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestD1 = newD1
            bestD2 = newD2
          }
        }
      }
      trace.controlParams[0] = origParams[0]
      trace.controlParams[1] = origParams[1]

      // Try bendSign with multiple deltas
      for (const bendDelta of [0.2, -0.2, 0.5, -0.5, 1, -1]) {
        trace.controlParams[2] = origParams[2] + bendDelta
        this.updateTracePoints(i)
        this.updateSingleTraceSample(i)
        const cost = this.computeCostForTrace(i)
        if (cost < bestCost) {
          bestCost = cost
          bestBend = trace.controlParams[2]
          bestD1 = origParams[0]
          bestD2 = origParams[1]
        }
      }

      // Apply best found parameters
      trace.controlParams[0] = bestD1
      trace.controlParams[1] = bestD2
      trace.controlParams[2] = bestBend
      this.updateTracePoints(i)
      this.updateSingleTraceSample(i)
    }

    // Update collision pairs less frequently
    if (this.optimizationStep % 5 === 0) {
      this.updateCollisionPairs()
    }
  }

  private buildOutputTraces() {
    // For angled traces, use the original points directly to preserve 45° angles
    // (don't use samplePolyline which would create intermediate points at invalid angles)
    this.outputTraces = this.traces.map((trace) => ({
      waypointPair: trace.waypointPair,
      points: [...trace.points], // Copy the original points
      networkId: trace.networkId,
    }))
  }

  override _step() {
    if (this.traces.length === 0) {
      this.effectiveTraceToTraceSpacing =
        this.problem.preferredTraceToTraceSpacing * 3
      this.initializeTraces()

      // Immediately resolve any intersections from direct routing
      for (let pass = 0; pass < 20; pass++) {
        const resolved = this.resolveIntersections()
        if (resolved === 0) break
        this.updateAllSampledTraces()
        this.updateCollisionPairs()
      }

      this.lastCost = this.computeTotalCost()
      this.stagnantSteps = 0

      // Fast path: if no collisions after initial resolution, skip optimization
      if (this.lastCost === 0 && this.findIntersectingPairs().length === 0) {
        this.optimizationStep = this.maxOptimizationSteps
      }
    }

    if (this.optimizationStep < this.maxOptimizationSteps) {
      const progress = this.optimizationStep / this.maxOptimizationSteps
      const startMultiplier = 3.0
      const endMultiplier = 1.0
      this.effectiveTraceToTraceSpacing =
        this.problem.preferredTraceToTraceSpacing *
        (startMultiplier + (endMultiplier - startMultiplier) * progress)

      this.optimizeStep()
      this.optimizationStep++

      // Resolve intersections every 10 steps (was 5)
      if (this.optimizationStep % 10 === 0) {
        const resolved = this.resolveIntersections()
        if (resolved > 0) {
          this.updateCollisionPairs()
        }
      }

      const currentCost = this.computeTotalCost()

      // Aggressive early termination when cost is very low
      if (currentCost < 1 && this.findIntersectingPairs().length === 0) {
        this.optimizationStep = this.maxOptimizationSteps
      } else if (currentCost >= this.lastCost * 0.99) {
        this.stagnantSteps++
        if (this.stagnantSteps > 5) {
          this.resolveIntersections()
          this.stagnantSteps = 0
        }
        if (this.stagnantSteps > 8) {
          this.optimizationStep = this.maxOptimizationSteps
        }
      } else {
        this.stagnantSteps = 0
      }
      this.lastCost = currentCost
    }

    if (this.optimizationStep >= this.maxOptimizationSteps) {
      // Final pass: update samples and resolve any remaining intersections
      this.updateAllSampledTraces()
      this.updateCollisionPairs()
      for (let i = 0; i < 30; i++) {
        const resolved = this.resolveIntersections()
        if (resolved === 0) break
        this.updateAllSampledTraces()
        this.updateCollisionPairs()
      }
      this.buildOutputTraces()
      this.solved = true
    }
  }

  override visualize(): GraphicsObject {
    if (this.traces.length > 0) {
      this.buildOutputTraces()
    }
    return visualizeCurvyTraceProblem(this.problem, this.outputTraces)
  }
}
