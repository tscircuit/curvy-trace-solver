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
 * Compute the 45-degree angled trace points from control parameters.
 * The trace goes:
 * 1. From start, perpendicular to boundary for distance d1 -> Turn1
 * 2. From Turn1, at 45 degrees toward the direction needed -> Midpoint(s)
 * 3. From last midpoint to Turn2, which is d2 away from end along perpDir2
 * 4. From Turn2 to end
 *
 * bendSign determines which 45-degree direction to prefer.
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

  // Turn1: go perpendicular from start for distance d1
  const turn1: Point = {
    x: start.x + d1 * perpDir1.x,
    y: start.y + d1 * perpDir1.y,
  }

  // Turn2: go perpendicular from end for distance d2 (backwards)
  const turn2: Point = {
    x: end.x + d2 * perpDir2.x,
    y: end.y + d2 * perpDir2.y,
  }

  // Vector from turn1 to turn2
  const dx = turn2.x - turn1.x
  const dy = turn2.y - turn1.y

  // If turn1 and turn2 are very close, just return a simple path
  if (Math.hypot(dx, dy) < 1e-6) {
    return [start, turn1, turn2, end]
  }

  // For 45-degree routing, we need to decompose the movement into
  // diagonal (45°) and orthogonal (horizontal or vertical) segments.
  // The bendSign determines whether we go diagonal-first or orthogonal-first.

  // Compute the 45-degree diagonal component
  // A 45-degree line has |dx| = |dy|
  const absDx = Math.abs(dx)
  const absDy = Math.abs(dy)

  // Determine the diagonal length (minimum of |dx| and |dy| for a true 45°)
  const diagLen = Math.min(absDx, absDy)

  // Diagonal direction (normalized to 45 degrees)
  const diagDirX = Math.sign(dx) * Math.SQRT1_2
  const diagDirY = Math.sign(dy) * Math.SQRT1_2

  // Orthogonal direction (the remaining movement after diagonal)
  let orthoX = 0
  let orthoY = 0
  if (absDx > absDy) {
    // More horizontal movement needed
    orthoX = dx - Math.sign(dx) * diagLen
  } else {
    // More vertical movement needed
    orthoY = dy - Math.sign(dy) * diagLen
  }

  // bendSign > 0: diagonal first, then orthogonal
  // bendSign < 0: orthogonal first, then diagonal
  // bendSign = 0: blend (we'll default to diagonal first)

  const normalizedBend = Math.tanh(bendSign) // Normalize to [-1, 1]

  let mid1: Point
  let mid2: Point | null = null

  if (normalizedBend >= 0) {
    // Diagonal first, then orthogonal
    mid1 = {
      x: turn1.x + diagDirX * diagLen * Math.SQRT2,
      y: turn1.y + diagDirY * diagLen * Math.SQRT2,
    }
    // If there's orthogonal movement needed, add another point
    if (Math.abs(orthoX) > 1e-6 || Math.abs(orthoY) > 1e-6) {
      mid2 = {
        x: mid1.x + orthoX,
        y: mid1.y + orthoY,
      }
    }
  } else {
    // Orthogonal first, then diagonal
    mid1 = {
      x: turn1.x + orthoX,
      y: turn1.y + orthoY,
    }
    // Then diagonal
    if (diagLen > 1e-6) {
      mid2 = {
        x: mid1.x + diagDirX * diagLen * Math.SQRT2,
        y: mid1.y + diagDirY * diagLen * Math.SQRT2,
      }
    }
  }

  // Clamp all points to bounds
  const clamp = (p: Point): Point => ({
    x: Math.max(minX, Math.min(maxX, p.x)),
    y: Math.max(minY, Math.min(maxY, p.y)),
  })

  const points = [start, clamp(turn1)]
  if (mid1) points.push(clamp(mid1))
  if (mid2) points.push(clamp(mid2))
  points.push(clamp(turn2), end)

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
    this.outputTraces = this.traces.map((trace) => ({
      waypointPair: trace.waypointPair,
      points: samplePolyline(trace.points, OUTPUT_SAMPLES),
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
