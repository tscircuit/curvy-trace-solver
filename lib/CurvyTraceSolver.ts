import { BaseSolver } from "@tscircuit/solver-utils"
import type { CurvyTraceProblem, OutputTrace, WaypointPair } from "./types"
import type { GraphicsObject } from "graphics-debug"
import type { Point, Bounds } from "@tscircuit/math-utils"
import { visualizeCurvyTraceProblem } from "./visualization-utils"
import { getObstacleOuterSegments } from "./geometry/getObstacleOuterSegments"

interface TraceWithControlPoints {
  waypointPair: WaypointPair
  ctrl1: Point
  ctrl2: Point
  networkId?: string
  t1: number
  t2: number
  // Perpendicular constraint data
  perpDir1: Point // Inward perpendicular direction at start
  perpDir2: Point // Inward perpendicular direction at end
  d1: number // Distance along perpDir1 from start to ctrl1
  d2: number // Distance along perpDir2 from end to ctrl2
  containedBy: number[] // Indices of traces that contain this trace
  contains: number[] // Indices of traces this trace contains
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
// Returns a unit vector pointing inward, perpendicular to the boundary edge
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

// Inline bezier sampling for performance - returns points directly into provided array
function sampleCubicBezierInline(
  p0x: number,
  p0y: number,
  p1x: number,
  p1y: number,
  p2x: number,
  p2y: number,
  p3x: number,
  p3y: number,
  points: Float64Array,
  numSamples: number,
): void {
  for (let i = 0; i <= numSamples; i++) {
    const t = i / numSamples
    const mt = 1 - t
    const mt2 = mt * mt
    const mt3 = mt2 * mt
    const t2 = t * t
    const t3 = t2 * t
    const idx = i * 2
    points[idx] = mt3 * p0x + 3 * mt2 * t * p1x + 3 * mt * t2 * p2x + t3 * p3x
    points[idx + 1] =
      mt3 * p0y + 3 * mt2 * t * p1y + 3 * mt * t2 * p2y + t3 * p3y
  }
}

// Sample a cubic bezier curve into Point array
function sampleCubicBezier(
  p0: Point,
  p1: Point,
  p2: Point,
  p3: Point,
  numSamples: number,
): Point[] {
  const points: Point[] = []
  for (let i = 0; i <= numSamples; i++) {
    const t = i / numSamples
    const mt = 1 - t
    const mt2 = mt * mt
    const mt3 = mt2 * mt
    const t2 = t * t
    const t3 = t2 * t
    points.push({
      x: mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x,
      y: mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y,
    })
  }
  return points
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

// Inlined segment-to-segment distance squared for hot path
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

// Check if two segments actually intersect (not just close)
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

// Compute bounding box for a sampled trace
function computeTraceBounds(
  points: Float64Array,
  numPoints: number,
): { minX: number; maxX: number; minY: number; maxY: number } {
  let minX = Infinity,
    maxX = -Infinity,
    minY = Infinity,
    maxY = -Infinity
  for (let i = 0; i < numPoints; i++) {
    const x = points[i * 2],
      y = points[i * 2 + 1]
    if (x < minX) minX = x
    if (x > maxX) maxX = x
    if (y < minY) minY = y
    if (y > maxY) maxY = y
  }
  return { minX, maxX, minY, maxY }
}

const OPT_SAMPLES = 5 // Samples during optimization
const OUTPUT_SAMPLES = 20 // Samples for final output

export class CurvyTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traces: TraceWithControlPoints[] = []
  private optimizationStep = 0
  private readonly maxOptimizationSteps = 100

  // Typed arrays for faster sampling
  private sampledPoints: Float64Array[] = []
  private traceBounds: {
    minX: number
    maxX: number
    minY: number
    maxY: number
  }[] = []

  // Pre-computed obstacle segments as flat arrays
  private obstacleSegments: Float64Array = new Float64Array(0)
  private obstacleNetworkIds: (string | undefined)[] = []
  private numObstacleSegments = 0

  // Collision pairs cache - which traces can possibly collide
  private collisionPairs: [number, number][] = []

  private lastCost = Infinity
  private stagnantSteps = 0

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

    // Compute containment relationships between all traces
    const containedBy: number[][] = tracesWithT.map(() => [])
    const contains: number[][] = tracesWithT.map(() => [])

    for (const trace of tracesWithT) {
      for (const other of tracesWithT) {
        if (trace.idx !== other.idx) {
          // Skip if same network
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

    // Compute nesting depth (how many traces contain this one)
    const nestingDepth = containedBy.map((arr) => arr.length)
    const maxDepth = Math.max(...nestingDepth, 1)

    this.traces = tracesWithT.map(({ pair, t1, t2, idx }) => {
      // Get inward perpendicular directions at start and end
      const perpDir1 = getInwardPerpendicular(pair.start, bounds)
      const perpDir2 = getInwardPerpendicular(pair.end, bounds)

      // Calculate chord length for initial distance estimation
      const chordLength = Math.hypot(
        pair.end.x - pair.start.x,
        pair.end.y - pair.start.y,
      )

      // Compute nesting depth for this trace
      const depth = nestingDepth[idx]
      const normalizedDepth = depth / maxDepth

      // Compute spatial depth based on midpoint distance to center
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

      // Initial perpendicular distances: proportional to chord length
      // Nested traces get smaller distances, traces near edges get larger distances
      const baseFactor = 0.25 + spatialDepth * 0.15
      const depthAdjustment = 1 - normalizedDepth * 0.3
      const initialDist = chordLength * baseFactor * depthAdjustment

      // Ensure minimum distance for reasonable curve shape
      const minDist = Math.min(W, H) * 0.05
      const d1 = Math.max(minDist, initialDist)
      const d2 = Math.max(minDist, initialDist)

      // Compute control points from perpendicular directions and distances
      const ctrl1 = {
        x: pair.start.x + d1 * perpDir1.x,
        y: pair.start.y + d1 * perpDir1.y,
      }
      const ctrl2 = {
        x: pair.end.x + d2 * perpDir2.x,
        y: pair.end.y + d2 * perpDir2.y,
      }

      return {
        waypointPair: pair,
        ctrl1,
        ctrl2,
        networkId: pair.networkId,
        t1,
        t2,
        perpDir1,
        perpDir2,
        d1,
        d2,
        containedBy: containedBy[idx],
        contains: contains[idx],
      }
    })

    // Initialize typed arrays for sampling
    this.sampledPoints = this.traces.map(
      () => new Float64Array((OPT_SAMPLES + 1) * 2),
    )
    this.traceBounds = this.traces.map(() => ({
      minX: 0,
      maxX: 0,
      minY: 0,
      maxY: 0,
    }))

    this.updateSampledTraces()
    this.updateCollisionPairs()
  }

  private updateSampledTraces() {
    for (let i = 0; i < this.traces.length; i++) {
      const trace = this.traces[i]
      sampleCubicBezierInline(
        trace.waypointPair.start.x,
        trace.waypointPair.start.y,
        trace.ctrl1.x,
        trace.ctrl1.y,
        trace.ctrl2.x,
        trace.ctrl2.y,
        trace.waypointPair.end.x,
        trace.waypointPair.end.y,
        this.sampledPoints[i],
        OPT_SAMPLES,
      )
      this.traceBounds[i] = computeTraceBounds(
        this.sampledPoints[i],
        OPT_SAMPLES + 1,
      )
    }
  }

  private updateSingleTraceSample(i: number) {
    const trace = this.traces[i]
    sampleCubicBezierInline(
      trace.waypointPair.start.x,
      trace.waypointPair.start.y,
      trace.ctrl1.x,
      trace.ctrl1.y,
      trace.ctrl2.x,
      trace.ctrl2.y,
      trace.waypointPair.end.x,
      trace.waypointPair.end.y,
      this.sampledPoints[i],
      OPT_SAMPLES,
    )
    this.traceBounds[i] = computeTraceBounds(
      this.sampledPoints[i],
      OPT_SAMPLES + 1,
    )
  }

  // Update control points from perpendicular distances
  private updateControlPointsFromDistances(i: number) {
    const trace = this.traces[i]
    trace.ctrl1.x = trace.waypointPair.start.x + trace.d1 * trace.perpDir1.x
    trace.ctrl1.y = trace.waypointPair.start.y + trace.d1 * trace.perpDir1.y
    trace.ctrl2.x = trace.waypointPair.end.x + trace.d2 * trace.perpDir2.x
    trace.ctrl2.y = trace.waypointPair.end.y + trace.d2 * trace.perpDir2.y
  }

  // Determine which trace pairs could possibly collide based on bounding boxes
  private updateCollisionPairs() {
    const { preferredTraceToTraceSpacing: preferredSpacing } = this.problem
    this.collisionPairs = []

    for (let i = 0; i < this.traces.length; i++) {
      for (let j = i + 1; j < this.traces.length; j++) {
        const ti = this.traces[i],
          tj = this.traces[j]
        if (ti.networkId && tj.networkId && ti.networkId === tj.networkId)
          continue

        const bi = this.traceBounds[i],
          bj = this.traceBounds[j]
        // Check if bounding boxes (expanded by preferredSpacing) overlap
        if (
          bi.maxX + preferredSpacing >= bj.minX &&
          bj.maxX + preferredSpacing >= bi.minX &&
          bi.maxY + preferredSpacing >= bj.minY &&
          bj.maxY + preferredSpacing >= bi.minY
        ) {
          this.collisionPairs.push([i, j])
        }
      }
    }
  }

  private computeTotalCost(): number {
    const { preferredTraceToTraceSpacing, preferredObstacleToTraceSpacing } =
      this.problem
    const traceSpacingSq = preferredTraceToTraceSpacing ** 2
    const obstacleSpacingSq = preferredObstacleToTraceSpacing ** 2
    let cost = 0

    // Cost between traces using collision pairs
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
            cost += (preferredTraceToTraceSpacing - dist) ** 2
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

        // Quick bounds check
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
    const { preferredTraceToTraceSpacing, preferredObstacleToTraceSpacing } =
      this.problem
    const traceSpacingSq = preferredTraceToTraceSpacing ** 2
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
      // Bounding box check
      if (
        bi.maxX + preferredTraceToTraceSpacing < bj.minX ||
        bj.maxX + preferredTraceToTraceSpacing < bi.minX ||
        bi.maxY + preferredTraceToTraceSpacing < bj.minY ||
        bj.maxY + preferredTraceToTraceSpacing < bi.minY
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
            cost += (preferredTraceToTraceSpacing - dist) ** 2
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

  // Check if two traces intersect or are very close (potential intersection)
  // Uses higher resolution sampling for accuracy
  private tracesIntersect(i: number, j: number): boolean {
    const trace1 = this.traces[i]
    const trace2 = this.traces[j]

    // Sample at higher resolution for intersection check
    const INTERSECT_SAMPLES = 15
    const p1 = new Float64Array((INTERSECT_SAMPLES + 1) * 2)
    const p2 = new Float64Array((INTERSECT_SAMPLES + 1) * 2)

    sampleCubicBezierInline(
      trace1.waypointPair.start.x,
      trace1.waypointPair.start.y,
      trace1.ctrl1.x,
      trace1.ctrl1.y,
      trace1.ctrl2.x,
      trace1.ctrl2.y,
      trace1.waypointPair.end.x,
      trace1.waypointPair.end.y,
      p1,
      INTERSECT_SAMPLES,
    )
    sampleCubicBezierInline(
      trace2.waypointPair.start.x,
      trace2.waypointPair.start.y,
      trace2.ctrl1.x,
      trace2.ctrl1.y,
      trace2.ctrl2.x,
      trace2.ctrl2.y,
      trace2.waypointPair.end.x,
      trace2.waypointPair.end.y,
      p2,
      INTERSECT_SAMPLES,
    )

    // Check for actual intersections only (to match scorer's threshold)
    for (let a = 0; a < INTERSECT_SAMPLES; a++) {
      const a1x = p1[a * 2]
      const a1y = p1[a * 2 + 1]
      const a2x = p1[(a + 1) * 2]
      const a2y = p1[(a + 1) * 2 + 1]

      for (let b = 0; b < INTERSECT_SAMPLES; b++) {
        const b1x = p2[b * 2]
        const b1y = p2[b * 2 + 1]
        const b2x = p2[(b + 1) * 2]
        const b2y = p2[(b + 1) * 2 + 1]

        // Check for actual segment intersection
        if (segmentsIntersect(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)) {
          return true
        }
      }
    }
    return false
  }

  // Find all pairs of traces that currently intersect
  private findIntersectingPairs(): [number, number][] {
    const intersecting: [number, number][] = []

    for (let i = 0; i < this.traces.length; i++) {
      for (let j = i + 1; j < this.traces.length; j++) {
        const ti = this.traces[i]
        const tj = this.traces[j]

        // Skip if same network
        if (ti.networkId && tj.networkId && ti.networkId === tj.networkId) {
          continue
        }

        // Quick bounding box check
        const bi = this.traceBounds[i]
        const bj = this.traceBounds[j]
        if (
          bi.maxX < bj.minX ||
          bj.maxX < bi.minX ||
          bi.maxY < bj.minY ||
          bj.maxY < bi.minY
        ) {
          continue
        }

        if (this.tracesIntersect(i, j)) {
          intersecting.push([i, j])
        }
      }
    }

    return intersecting
  }

  // Resolve intersections by separating traces in depth
  // Only keeps changes if they improve overall cost
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

      // Skip if they no longer intersect (might have been fixed by earlier resolution)
      if (!this.tracesIntersect(i, j)) continue

      // Determine which trace should be outer (smaller depth) vs inner (larger depth)
      let outerIdx: number
      let innerIdx: number

      if (ti.containedBy.includes(j)) {
        // ti is contained by tj, so ti should be inner (go deeper)
        outerIdx = j
        innerIdx = i
      } else if (tj.containedBy.includes(i)) {
        // tj is contained by ti, so tj should be inner (go deeper)
        outerIdx = i
        innerIdx = j
      } else {
        // No containment relationship - the one with larger depth should go deeper
        const avgDi = (ti.d1 + ti.d2) / 2
        const avgDj = (tj.d1 + tj.d2) / 2
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
      const origOuterD1 = outerTrace.d1
      const origOuterD2 = outerTrace.d2
      const origInnerD1 = innerTrace.d1
      const origInnerD2 = innerTrace.d2
      const costBeforeThis = this.computeTotalCost()

      // Calculate the minimum separation needed
      const separation = preferredTraceToTraceSpacing * 2

      // Try strategies in order of aggressiveness
      const strategies = [
        // Strategy 1: Small increase for inner
        { innerMult: 1, outerMult: 0 },
        // Strategy 2: Small decrease for outer
        { innerMult: 0, outerMult: 1 },
        // Strategy 3: Both directions
        { innerMult: 0.5, outerMult: 0.5 },
        // Strategy 4: Larger increase for inner
        { innerMult: 2, outerMult: 0 },
        // Strategy 5: Both larger
        { innerMult: 1, outerMult: 1 },
        // More aggressive
        { innerMult: 3, outerMult: 0 },
        { innerMult: 2, outerMult: 1 },
        { innerMult: 4, outerMult: 0 },
      ]

      let bestCost = costBeforeThis
      let bestStrategy: { innerMult: number; outerMult: number } | null = null

      for (const strategy of strategies) {
        // Reset to original
        outerTrace.d1 = origOuterD1
        outerTrace.d2 = origOuterD2
        innerTrace.d1 = origInnerD1
        innerTrace.d2 = origInnerD2

        // Apply strategy
        innerTrace.d1 = Math.min(
          maxDist,
          origInnerD1 + separation * strategy.innerMult,
        )
        innerTrace.d2 = Math.min(
          maxDist,
          origInnerD2 + separation * strategy.innerMult,
        )
        outerTrace.d1 = Math.max(
          minDist,
          origOuterD1 - separation * strategy.outerMult,
        )
        outerTrace.d2 = Math.max(
          minDist,
          origOuterD2 - separation * strategy.outerMult,
        )
        this.updateControlPointsFromDistances(innerIdx)
        this.updateControlPointsFromDistances(outerIdx)
        this.updateSingleTraceSample(innerIdx)
        this.updateSingleTraceSample(outerIdx)

        // Check if intersection is resolved - prioritize eliminating intersections
        if (!this.tracesIntersect(outerIdx, innerIdx)) {
          const newCost = this.computeTotalCost()
          // Accept if it eliminates intersection - use smaller changes when possible
          if (bestStrategy === null || newCost < bestCost) {
            bestCost = newCost
            bestStrategy = strategy
          }
        }
      }

      // Apply best strategy or revert
      if (bestStrategy) {
        innerTrace.d1 = Math.min(
          maxDist,
          origInnerD1 + separation * bestStrategy.innerMult,
        )
        innerTrace.d2 = Math.min(
          maxDist,
          origInnerD2 + separation * bestStrategy.innerMult,
        )
        outerTrace.d1 = Math.max(
          minDist,
          origOuterD1 - separation * bestStrategy.outerMult,
        )
        outerTrace.d2 = Math.max(
          minDist,
          origOuterD2 - separation * bestStrategy.outerMult,
        )
        this.updateControlPointsFromDistances(innerIdx)
        this.updateControlPointsFromDistances(outerIdx)
        this.updateSingleTraceSample(innerIdx)
        this.updateSingleTraceSample(outerIdx)
        resolved++
      } else {
        // Revert to original
        outerTrace.d1 = origOuterD1
        outerTrace.d2 = origOuterD2
        innerTrace.d1 = origInnerD1
        innerTrace.d2 = origInnerD2
        this.updateControlPointsFromDistances(outerIdx)
        this.updateControlPointsFromDistances(innerIdx)
        this.updateSingleTraceSample(outerIdx)
        this.updateSingleTraceSample(innerIdx)
      }
    }

    return resolved
  }

  private optimizeStep() {
    const { bounds, preferredTraceToTraceSpacing } = this.problem
    const { minX, maxX, minY, maxY } = bounds
    const minDim = Math.min(maxX - minX, maxY - minY)

    // Adaptive step size for perpendicular distances
    const progress = this.optimizationStep / this.maxOptimizationSteps
    const baseStep = 4.0 * (1 - progress) + 0.5

    // Minimum and maximum perpendicular distances
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

      // For high-cost traces, use larger steps
      const costMultiplier = currentCost > 100 ? 2.0 : 1.0
      const steps = [baseStep * costMultiplier, baseStep * 1.5 * costMultiplier, baseStep * 0.5]

      for (const step of steps) {
        // Distance deltas to try - include larger jumps for high cost traces
        const deltas =
          currentCost > 100
            ? [
                step,
                -step,
                step * 2,
                -step * 2,
                step * 3,
                -step * 3,
                preferredTraceToTraceSpacing * 2,
                -preferredTraceToTraceSpacing * 2,
              ]
            : [step, -step, step * 2, -step * 2]

        let bestCost = this.computeCostForTrace(i)
        let bestD1 = trace.d1
        let bestD2 = trace.d2

        const origD1 = trace.d1
        const origD2 = trace.d2

        for (const delta of deltas) {
          // Try adjusting d1 (control point 1 distance)
          trace.d1 = Math.max(minDist, Math.min(maxDist, origD1 + delta))
          this.updateControlPointsFromDistances(i)
          this.updateSingleTraceSample(i)
          const cost1 = this.computeCostForTrace(i)
          if (cost1 < bestCost) {
            bestCost = cost1
            bestD1 = trace.d1
            bestD2 = origD2
          }
          trace.d1 = origD1
          this.updateControlPointsFromDistances(i)

          // Try adjusting d2 (control point 2 distance)
          trace.d2 = Math.max(minDist, Math.min(maxDist, origD2 + delta))
          this.updateControlPointsFromDistances(i)
          this.updateSingleTraceSample(i)
          const cost2 = this.computeCostForTrace(i)
          if (cost2 < bestCost) {
            bestCost = cost2
            bestD1 = origD1
            bestD2 = trace.d2
          }
          trace.d2 = origD2
          this.updateControlPointsFromDistances(i)

          // Try adjusting both together (same direction)
          trace.d1 = Math.max(minDist, Math.min(maxDist, origD1 + delta))
          trace.d2 = Math.max(minDist, Math.min(maxDist, origD2 + delta))
          this.updateControlPointsFromDistances(i)
          this.updateSingleTraceSample(i)
          const cost3 = this.computeCostForTrace(i)
          if (cost3 < bestCost) {
            bestCost = cost3
            bestD1 = trace.d1
            bestD2 = trace.d2
          }
          trace.d1 = origD1
          trace.d2 = origD2
          this.updateControlPointsFromDistances(i)

          // Try adjusting both in opposite directions
          trace.d1 = Math.max(minDist, Math.min(maxDist, origD1 + delta))
          trace.d2 = Math.max(minDist, Math.min(maxDist, origD2 - delta))
          this.updateControlPointsFromDistances(i)
          this.updateSingleTraceSample(i)
          const cost4 = this.computeCostForTrace(i)
          if (cost4 < bestCost) {
            bestCost = cost4
            bestD1 = trace.d1
            bestD2 = trace.d2
          }
          trace.d1 = origD1
          trace.d2 = origD2
          this.updateControlPointsFromDistances(i)
        }

        // Apply best found distances
        trace.d1 = bestD1
        trace.d2 = bestD2
        this.updateControlPointsFromDistances(i)
        this.updateSingleTraceSample(i)

        if (bestCost < currentCost * 0.9) break // Found significant improvement
      }
    }

    // Update collision pairs periodically
    if (this.optimizationStep % 10 === 0) {
      this.updateCollisionPairs()
    }
  }

  private buildOutputTraces() {
    this.outputTraces = this.traces.map((trace) => ({
      waypointPair: trace.waypointPair,
      points: sampleCubicBezier(
        trace.waypointPair.start,
        trace.ctrl1,
        trace.ctrl2,
        trace.waypointPair.end,
        OUTPUT_SAMPLES,
      ),
      networkId: trace.networkId,
    }))
  }

  override _step() {
    if (this.traces.length === 0) {
      this.initializeTraces()
      this.lastCost = this.computeTotalCost()
      this.stagnantSteps = 0
    }

    if (this.optimizationStep < this.maxOptimizationSteps) {
      this.optimizeStep()
      this.optimizationStep++

      // Periodically try to resolve intersections to help escape local minima
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
          // Try resolving intersections when stuck
          const resolved = this.resolveIntersections()
          if (resolved > 0) {
            this.updateCollisionPairs()
            this.stagnantSteps = 0 // Reset stagnation counter after successful resolution
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
      // Final pass to resolve any remaining intersections
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
