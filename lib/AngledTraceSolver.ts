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

// Point to segment distance squared
function ptSegDistSq(
  px: number,
  py: number,
  sx: number,
  sy: number,
  ex: number,
  ey: number,
): number {
  const dx = ex - sx
  const dy = ey - sy
  const lenSq = dx * dx + dy * dy
  if (lenSq === 0) {
    const dpx = px - sx
    const dpy = py - sy
    return dpx * dpx + dpy * dpy
  }
  const t = Math.max(0, Math.min(1, ((px - sx) * dx + (py - sy) * dy) / lenSq))
  const projX = sx + t * dx
  const projY = sy + t * dy
  const dpx = px - projX
  const dpy = py - projY
  return dpx * dpx + dpy * dpy
}

// Segment-to-segment distance squared
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
  // Check for intersection first
  const d1 = (b2x - b1x) * (a1y - b1y) - (b2y - b1y) * (a1x - b1x)
  const d2 = (b2x - b1x) * (a2y - b1y) - (b2y - b1y) * (a2x - b1x)
  const d3 = (a2x - a1x) * (b1y - a1y) - (a2y - a1y) * (b1x - a1x)
  const d4 = (a2x - a1x) * (b2y - a1y) - (a2y - a1y) * (b2x - a1x)

  if (
    ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
    ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
  ) {
    return 0
  }

  return Math.min(
    ptSegDistSq(a1x, a1y, b1x, b1y, b2x, b2y),
    ptSegDistSq(a2x, a2y, b1x, b1y, b2x, b2y),
    ptSegDistSq(b1x, b1y, a1x, a1y, a2x, a2y),
    ptSegDistSq(b2x, b2y, a1x, a1y, a2x, a2y),
  )
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
function computeTraceBounds(
  points: Point[],
): { minX: number; maxX: number; minY: number; maxY: number } {
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
  if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
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

  // Build the path
  const points: Point[] = [start, turn1]

  const hasDiagonal = diagDist > 1e-6
  const hasOrthogonal = Math.abs(orthoX) > 1e-6 || Math.abs(orthoY) > 1e-6

  if (hasDiagonal && hasOrthogonal) {
    // Need an intermediate point
    const normalizedBend = Math.tanh(bendSign)

    if (normalizedBend >= 0) {
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
  const filtered: Point[] = [points[0]]
  for (let i = 1; i < points.length; i++) {
    const prev = filtered[filtered.length - 1]
    const curr = points[i]
    if (Math.hypot(curr.x - prev.x, curr.y - prev.y) > 1e-6) {
      filtered.push(curr)
    }
  }

  return filtered
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
        const t =
          segmentLengths[i] > 1e-9 ? segDist / segmentLengths[i] : 0
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

const OPT_SAMPLES = 8 // More samples for angled traces since they have more segments
const OUTPUT_SAMPLES = 20

export class AngledTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traces: AngledTraceWithControlPoints[] = []
  private optimizationStep = 0
  private readonly maxOptimizationSteps = 100

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
    return this.traces.map((t) => [...t.controlParams] as [number, number, number])
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

    this.traces = tracesWithT.map(({ pair, t1, t2, idx }) => {
      const perpDir1 = getInwardPerpendicular(pair.start, bounds)
      const perpDir2 = getInwardPerpendicular(pair.end, bounds)

      // Calculate chord length for initial distance estimation
      const chordLength = Math.hypot(
        pair.end.x - pair.start.x,
        pair.end.y - pair.start.y,
      )

      const depth = nestingDepth[idx]
      const normalizedDepth = depth / maxDepth

      // Compute spatial depth
      const midPoint = {
        x: (pair.start.x + pair.end.x) / 2,
        y: (pair.start.y + pair.end.y) / 2,
      }
      const distToCenter = Math.hypot(
        midPoint.x - center.x,
        midPoint.y - center.y,
      )
      const maxDist = Math.hypot(W / 2, H / 2)
      const spatialDepth = 1 - distToCenter / maxDist

      // Initial perpendicular distances
      const baseFactor = 0.2 + spatialDepth * 0.1
      const depthAdjustment = 1 - normalizedDepth * 0.3
      const initialDist = chordLength * baseFactor * depthAdjustment

      const minDist = Math.min(W, H) * 0.03
      const d1 = Math.max(minDist, initialDist)
      const d2 = Math.max(minDist, initialDist)
      const bendSign = 0 // Start neutral

      // Compute initial points
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

    return cost
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
    const minDist = minDim * 0.02
    const maxDist = minDim * 1.5

    const intersecting = this.findIntersectingPairs()
    if (intersecting.length === 0) return 0

    let resolved = 0

    for (const [i, j] of intersecting) {
      const ti = this.traces[i]
      const tj = this.traces[j]

      if (!this.tracesIntersect(i, j)) continue

      // Determine outer vs inner trace
      let outerIdx: number
      let innerIdx: number

      if (ti.containedBy.includes(j)) {
        outerIdx = j
        innerIdx = i
      } else if (tj.containedBy.includes(i)) {
        outerIdx = i
        innerIdx = j
      } else {
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
      const costBefore = this.computeTotalCost()

      const separation = preferredTraceToTraceSpacing * 2

      const strategies = [
        { innerMult: 1, outerMult: 0, bendDelta: 0 },
        { innerMult: 0, outerMult: 1, bendDelta: 0 },
        { innerMult: 0.5, outerMult: 0.5, bendDelta: 0 },
        { innerMult: 2, outerMult: 0, bendDelta: 0 },
        { innerMult: 0, outerMult: 0, bendDelta: 1 }, // Try bending
        { innerMult: 0, outerMult: 0, bendDelta: -1 },
        { innerMult: 1, outerMult: 0, bendDelta: 1 },
        { innerMult: 1, outerMult: 0, bendDelta: -1 },
        { innerMult: 3, outerMult: 0, bendDelta: 0 },
        { innerMult: 2, outerMult: 1, bendDelta: 0 },
      ]

      let bestCost = costBefore
      let bestStrategy: (typeof strategies)[0] | null = null

      for (const strategy of strategies) {
        // Reset
        outerTrace.controlParams = [...origOuter]
        innerTrace.controlParams = [...origInner]

        // Apply strategy
        innerTrace.controlParams[0] = Math.min(
          maxDist,
          origInner[0] + separation * strategy.innerMult,
        )
        innerTrace.controlParams[1] = Math.min(
          maxDist,
          origInner[1] + separation * strategy.innerMult,
        )
        outerTrace.controlParams[0] = Math.max(
          minDist,
          origOuter[0] - separation * strategy.outerMult,
        )
        outerTrace.controlParams[1] = Math.max(
          minDist,
          origOuter[1] - separation * strategy.outerMult,
        )
        innerTrace.controlParams[2] += strategy.bendDelta

        this.updateTracePoints(innerIdx)
        this.updateTracePoints(outerIdx)
        this.updateSingleTraceSample(innerIdx)
        this.updateSingleTraceSample(outerIdx)

        if (!this.tracesIntersect(outerIdx, innerIdx)) {
          const newCost = this.computeTotalCost()
          if (bestStrategy === null || newCost < bestCost) {
            bestCost = newCost
            bestStrategy = strategy
          }
        }
      }

      if (bestStrategy) {
        innerTrace.controlParams[0] = Math.min(
          maxDist,
          origInner[0] + separation * bestStrategy.innerMult,
        )
        innerTrace.controlParams[1] = Math.min(
          maxDist,
          origInner[1] + separation * bestStrategy.innerMult,
        )
        outerTrace.controlParams[0] = Math.max(
          minDist,
          origOuter[0] - separation * bestStrategy.outerMult,
        )
        outerTrace.controlParams[1] = Math.max(
          minDist,
          origOuter[1] - separation * bestStrategy.outerMult,
        )
        innerTrace.controlParams[2] = origInner[2] + bestStrategy.bendDelta

        this.updateTracePoints(innerIdx)
        this.updateTracePoints(outerIdx)
        this.updateSingleTraceSample(innerIdx)
        this.updateSingleTraceSample(outerIdx)
        resolved++
      } else {
        // Revert
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
    const effectiveSpacing = this.effectiveTraceToTraceSpacing

    const progress = this.optimizationStep / this.maxOptimizationSteps
    const baseStep = 4.0 * (1 - progress) + 0.5

    const minDist = minDim * 0.02
    const maxDist = minDim * 1.5

    // Sort traces by cost (worst first)
    const traceCosts: { idx: number; cost: number }[] = []
    for (let i = 0; i < this.traces.length; i++) {
      traceCosts.push({ idx: i, cost: this.computeCostForTrace(i) })
    }
    traceCosts.sort((a, b) => b.cost - a.cost)

    for (const { idx: i, cost: currentCost } of traceCosts) {
      if (currentCost === 0) continue

      const trace = this.traces[i]
      const costMultiplier = currentCost > 100 ? 2.0 : 1.0
      const steps = [
        baseStep * costMultiplier,
        baseStep * 1.5 * costMultiplier,
        baseStep * 0.5,
      ]

      for (const step of steps) {
        const deltas =
          currentCost > 100
            ? [step, -step, step * 2, -step * 2, step * 3, -step * 3]
            : [step, -step, step * 2, -step * 2]

        const bendDeltas = [0.2, -0.2, 0.5, -0.5, 1, -1]

        let bestCost = this.computeCostForTrace(i)
        const bestParams = [...trace.controlParams] as [number, number, number]
        const origParams = [...trace.controlParams] as [number, number, number]

        // Try d1 adjustments
        for (const delta of deltas) {
          trace.controlParams[0] = Math.max(
            minDist,
            Math.min(maxDist, origParams[0] + delta),
          )
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestParams[0] = trace.controlParams[0]
            bestParams[1] = origParams[1]
            bestParams[2] = origParams[2]
          }
          trace.controlParams[0] = origParams[0]
        }

        // Try d2 adjustments
        for (const delta of deltas) {
          trace.controlParams[1] = Math.max(
            minDist,
            Math.min(maxDist, origParams[1] + delta),
          )
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestParams[0] = origParams[0]
            bestParams[1] = trace.controlParams[1]
            bestParams[2] = origParams[2]
          }
          trace.controlParams[1] = origParams[1]
        }

        // Try both d1 and d2 together
        for (const delta of deltas) {
          trace.controlParams[0] = Math.max(
            minDist,
            Math.min(maxDist, origParams[0] + delta),
          )
          trace.controlParams[1] = Math.max(
            minDist,
            Math.min(maxDist, origParams[1] + delta),
          )
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestParams[0] = trace.controlParams[0]
            bestParams[1] = trace.controlParams[1]
            bestParams[2] = origParams[2]
          }
          trace.controlParams[0] = origParams[0]
          trace.controlParams[1] = origParams[1]
        }

        // Try bendSign adjustments
        for (const bendDelta of bendDeltas) {
          trace.controlParams[2] = origParams[2] + bendDelta
          this.updateTracePoints(i)
          this.updateSingleTraceSample(i)
          const cost = this.computeCostForTrace(i)
          if (cost < bestCost) {
            bestCost = cost
            bestParams[0] = origParams[0]
            bestParams[1] = origParams[1]
            bestParams[2] = trace.controlParams[2]
          }
          trace.controlParams[2] = origParams[2]
        }

        // Apply best found parameters
        trace.controlParams = bestParams
        this.updateTracePoints(i)
        this.updateSingleTraceSample(i)

        if (bestCost < currentCost * 0.9) break
      }
    }

    if (this.optimizationStep % 10 === 0) {
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
      this.lastCost = this.computeTotalCost()
      this.stagnantSteps = 0
    }

    if (this.optimizationStep < this.maxOptimizationSteps) {
      const progress = this.optimizationStep / this.maxOptimizationSteps
      const startMultiplier = 3.0
      const endMultiplier = 1.0
      const currentMultiplier =
        startMultiplier + (endMultiplier - startMultiplier) * progress
      this.effectiveTraceToTraceSpacing =
        this.problem.preferredTraceToTraceSpacing * currentMultiplier

      this.optimizeStep()
      this.optimizationStep++

      if (this.optimizationStep % 10 === 0) {
        const resolved = this.resolveIntersections()
        if (resolved > 0) {
          this.updateCollisionPairs()
        }
      }

      const currentCost = this.computeTotalCost()
      if (currentCost === 0) {
        this.optimizationStep = this.maxOptimizationSteps
      } else if (currentCost >= this.lastCost * 0.99) {
        this.stagnantSteps++
        if (this.stagnantSteps > 10) {
          const resolved = this.resolveIntersections()
          if (resolved > 0) {
            this.updateCollisionPairs()
            this.stagnantSteps = 0
          } else if (this.stagnantSteps > 15) {
            this.optimizationStep = this.maxOptimizationSteps
          }
        }
      } else {
        this.stagnantSteps = 0
      }
      this.lastCost = currentCost
    }

    if (this.optimizationStep >= this.maxOptimizationSteps) {
      this.resolveIntersections()
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
