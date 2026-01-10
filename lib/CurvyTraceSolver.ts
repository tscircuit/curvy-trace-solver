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

// Get point on perimeter at position t
function getPerimeterPoint(t: number, bounds: Bounds): Point {
  const { minX, maxX, minY, maxY } = bounds
  const W = maxX - minX
  const H = maxY - minY
  const perimeter = 2 * W + 2 * H
  t = ((t % perimeter) + perimeter) % perimeter

  if (t <= W) return { x: minX + t, y: maxY }
  if (t <= W + H) return { x: maxX, y: maxY - (t - W) }
  if (t <= 2 * W + H) return { x: maxX - (t - W - H), y: minY }
  return { x: minX, y: minY + (t - 2 * W - H) }
}

// Inline bezier sampling for performance - returns points directly into provided array
function sampleCubicBezierInline(
  p0x: number, p0y: number,
  p1x: number, p1y: number,
  p2x: number, p2y: number,
  p3x: number, p3y: number,
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
    points[idx + 1] = mt3 * p0y + 3 * mt2 * t * p1y + 3 * mt * t2 * p2y + t3 * p3y
  }
}

// Sample a cubic bezier curve into Point array
function sampleCubicBezier(
  p0: Point, p1: Point, p2: Point, p3: Point, numSamples: number,
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
function ptSegDistSq(px: number, py: number, sx: number, sy: number, ex: number, ey: number): number {
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
  a1x: number, a1y: number, a2x: number, a2y: number,
  b1x: number, b1y: number, b2x: number, b2y: number,
): number {
  // Check for intersection first
  const d1 = (b2x - b1x) * (a1y - b1y) - (b2y - b1y) * (a1x - b1x)
  const d2 = (b2x - b1x) * (a2y - b1y) - (b2y - b1y) * (a2x - b1x)
  const d3 = (a2x - a1x) * (b1y - a1y) - (a2y - a1y) * (b1x - a1x)
  const d4 = (a2x - a1x) * (b2y - a1y) - (a2y - a1y) * (b2x - a1x)

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return 0
  }

  return Math.min(
    ptSegDistSq(a1x, a1y, b1x, b1y, b2x, b2y),
    ptSegDistSq(a2x, a2y, b1x, b1y, b2x, b2y),
    ptSegDistSq(b1x, b1y, a1x, a1y, a2x, a2y),
    ptSegDistSq(b2x, b2y, a1x, a1y, a2x, a2y),
  )
}

// Check if chord A contains chord B
function chordContains(
  aT1: number, aT2: number, bT1: number, bT2: number, perimeter: number,
): boolean {
  const normalize = (t: number) => ((t % perimeter) + perimeter) % perimeter
  const a1 = normalize(aT1), a2 = normalize(aT2)
  const b1 = normalize(bT1), b2 = normalize(bT2)
  const [aMin, aMax] = a1 < a2 ? [a1, a2] : [a2, a1]
  return b1 > aMin && b1 < aMax && b2 > aMin && b2 < aMax
}

function getBoundsCenter(bounds: Bounds): Point {
  return { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 }
}

// Compute bounding box for a sampled trace
function computeTraceBounds(points: Float64Array, numPoints: number): { minX: number, maxX: number, minY: number, maxY: number } {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  for (let i = 0; i < numPoints; i++) {
    const x = points[i * 2], y = points[i * 2 + 1]
    if (x < minX) minX = x
    if (x > maxX) maxX = x
    if (y < minY) minY = y
    if (y > maxY) maxY = y
  }
  return { minX, maxX, minY, maxY }
}

const OPT_SAMPLES = 10  // Samples during optimization
const OUTPUT_SAMPLES = 20  // Samples for final output

export class CurvyTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traces: TraceWithControlPoints[] = []
  private optimizationStep = 0
  private readonly maxOptimizationSteps = 100

  // Typed arrays for faster sampling
  private sampledPoints: Float64Array[] = []
  private traceBounds: { minX: number, maxX: number, minY: number, maxY: number }[] = []

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

    const nestingDepth = new Map<number, number>()
    for (const trace of tracesWithT) {
      let depth = 0
      for (const other of tracesWithT) {
        if (trace.idx !== other.idx && chordContains(other.t1, other.t2, trace.t1, trace.t2, perimeter)) {
          depth++
        }
      }
      nestingDepth.set(trace.idx, depth)
    }

    const maxDepth = Math.max(...Array.from(nestingDepth.values()), 1)

    this.traces = tracesWithT.map(({ pair, t1, t2, idx }) => {
      let dt = t2 - t1
      if (dt > perimeter / 2) dt -= perimeter
      if (dt < -perimeter / 2) dt += perimeter

      const tCtrl1 = t1 + dt * 0.33
      const tCtrl2 = t1 + dt * 0.67

      const pPerim1 = getPerimeterPoint(tCtrl1, bounds)
      const pPerim2 = getPerimeterPoint(tCtrl2, bounds)

      const pLinear1 = {
        x: pair.start.x + (pair.end.x - pair.start.x) * 0.33,
        y: pair.start.y + (pair.end.y - pair.start.y) * 0.33,
      }
      const pLinear2 = {
        x: pair.start.x + (pair.end.x - pair.start.x) * 0.67,
        y: pair.start.y + (pair.end.y - pair.start.y) * 0.67,
      }

      const midPoint = { x: (pair.start.x + pair.end.x) / 2, y: (pair.start.y + pair.end.y) / 2 }
      const distToCenter = Math.hypot(midPoint.x - center.x, midPoint.y - center.y)
      const maxDist = Math.hypot(W / 2, H / 2)
      const spatialDepth = 1 - distToCenter / maxDist

      const depth = nestingDepth.get(idx) || 0
      const normalizedDepth = depth / maxDepth

      const basePull = 0.3 + spatialDepth * 0.4
      const pullAmount = Math.max(0.05, basePull - normalizedDepth * 0.2)

      return {
        waypointPair: pair,
        ctrl1: {
          x: pLinear1.x * (1 - pullAmount) + pPerim1.x * pullAmount,
          y: pLinear1.y * (1 - pullAmount) + pPerim1.y * pullAmount,
        },
        ctrl2: {
          x: pLinear2.x * (1 - pullAmount) + pPerim2.x * pullAmount,
          y: pLinear2.y * (1 - pullAmount) + pPerim2.y * pullAmount,
        },
        networkId: pair.networkId,
        t1,
        t2,
      }
    })

    // Initialize typed arrays for sampling
    this.sampledPoints = this.traces.map(() => new Float64Array((OPT_SAMPLES + 1) * 2))
    this.traceBounds = this.traces.map(() => ({ minX: 0, maxX: 0, minY: 0, maxY: 0 }))

    this.updateSampledTraces()
    this.updateCollisionPairs()
  }

  private updateSampledTraces() {
    for (let i = 0; i < this.traces.length; i++) {
      const trace = this.traces[i]
      sampleCubicBezierInline(
        trace.waypointPair.start.x, trace.waypointPair.start.y,
        trace.ctrl1.x, trace.ctrl1.y,
        trace.ctrl2.x, trace.ctrl2.y,
        trace.waypointPair.end.x, trace.waypointPair.end.y,
        this.sampledPoints[i],
        OPT_SAMPLES,
      )
      this.traceBounds[i] = computeTraceBounds(this.sampledPoints[i], OPT_SAMPLES + 1)
    }
  }

  private updateSingleTraceSample(i: number) {
    const trace = this.traces[i]
    sampleCubicBezierInline(
      trace.waypointPair.start.x, trace.waypointPair.start.y,
      trace.ctrl1.x, trace.ctrl1.y,
      trace.ctrl2.x, trace.ctrl2.y,
      trace.waypointPair.end.x, trace.waypointPair.end.y,
      this.sampledPoints[i],
      OPT_SAMPLES,
    )
    this.traceBounds[i] = computeTraceBounds(this.sampledPoints[i], OPT_SAMPLES + 1)
  }

  // Determine which trace pairs could possibly collide based on bounding boxes
  private updateCollisionPairs() {
    const { preferredSpacing } = this.problem
    this.collisionPairs = []

    for (let i = 0; i < this.traces.length; i++) {
      for (let j = i + 1; j < this.traces.length; j++) {
        const ti = this.traces[i], tj = this.traces[j]
        if (ti.networkId && tj.networkId && ti.networkId === tj.networkId) continue

        const bi = this.traceBounds[i], bj = this.traceBounds[j]
        // Check if bounding boxes (expanded by preferredSpacing) overlap
        if (bi.maxX + preferredSpacing >= bj.minX && bj.maxX + preferredSpacing >= bi.minX &&
            bi.maxY + preferredSpacing >= bj.minY && bj.maxY + preferredSpacing >= bi.minY) {
          this.collisionPairs.push([i, j])
        }
      }
    }
  }

  private computeTotalCost(): number {
    const { preferredSpacing } = this.problem
    const spacingSq = preferredSpacing * preferredSpacing
    let cost = 0

    // Cost between traces using collision pairs
    for (const [i, j] of this.collisionPairs) {
      const pi = this.sampledPoints[i], pj = this.sampledPoints[j]

      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2], a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2], a2y = pi[(a + 1) * 2 + 1]

        for (let b = 0; b < OPT_SAMPLES; b++) {
          const b1x = pj[b * 2], b1y = pj[b * 2 + 1]
          const b2x = pj[(b + 1) * 2], b2y = pj[(b + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
          if (distSq < spacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (preferredSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * spacingSq
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
        if (trace.networkId && this.obstacleNetworkIds[obsIdx] &&
            trace.networkId === this.obstacleNetworkIds[obsIdx]) continue

        const obsBase = obsIdx * 4
        const ox1 = this.obstacleSegments[obsBase]
        const oy1 = this.obstacleSegments[obsBase + 1]
        const ox2 = this.obstacleSegments[obsBase + 2]
        const oy2 = this.obstacleSegments[obsBase + 3]

        // Quick bounds check
        const obsMinX = Math.min(ox1, ox2), obsMaxX = Math.max(ox1, ox2)
        const obsMinY = Math.min(oy1, oy2), obsMaxY = Math.max(oy1, oy2)
        if (bi.maxX + preferredSpacing < obsMinX || obsMaxX + preferredSpacing < bi.minX ||
            bi.maxY + preferredSpacing < obsMinY || obsMaxY + preferredSpacing < bi.minY) continue

        for (let a = 0; a < OPT_SAMPLES; a++) {
          const a1x = pi[a * 2], a1y = pi[a * 2 + 1]
          const a2x = pi[(a + 1) * 2], a2y = pi[(a + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, ox1, oy1, ox2, oy2)
          if (distSq < spacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (preferredSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * spacingSq
          }
        }
      }
    }

    return cost
  }

  private computeCostForTrace(traceIdx: number): number {
    const { preferredSpacing } = this.problem
    const spacingSq = preferredSpacing * preferredSpacing
    const trace = this.traces[traceIdx]
    const pi = this.sampledPoints[traceIdx]
    const bi = this.traceBounds[traceIdx]
    let cost = 0

    // Cost against other traces
    for (let j = 0; j < this.traces.length; j++) {
      if (j === traceIdx) continue
      const other = this.traces[j]
      if (trace.networkId && other.networkId && trace.networkId === other.networkId) continue

      const bj = this.traceBounds[j]
      // Bounding box check
      if (bi.maxX + preferredSpacing < bj.minX || bj.maxX + preferredSpacing < bi.minX ||
          bi.maxY + preferredSpacing < bj.minY || bj.maxY + preferredSpacing < bi.minY) continue

      const pj = this.sampledPoints[j]
      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2], a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2], a2y = pi[(a + 1) * 2 + 1]

        for (let b = 0; b < OPT_SAMPLES; b++) {
          const b1x = pj[b * 2], b1y = pj[b * 2 + 1]
          const b2x = pj[(b + 1) * 2], b2y = pj[(b + 1) * 2 + 1]

          const distSq = segmentDistSq(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
          if (distSq < spacingSq) {
            const dist = Math.sqrt(distSq)
            cost += (preferredSpacing - dist) ** 2
            if (distSq < 1e-18) cost += 20 * spacingSq
          }
        }
      }
    }

    // Cost against obstacles
    for (let obsIdx = 0; obsIdx < this.numObstacleSegments; obsIdx++) {
      if (trace.networkId && this.obstacleNetworkIds[obsIdx] &&
          trace.networkId === this.obstacleNetworkIds[obsIdx]) continue

      const obsBase = obsIdx * 4
      const ox1 = this.obstacleSegments[obsBase]
      const oy1 = this.obstacleSegments[obsBase + 1]
      const ox2 = this.obstacleSegments[obsBase + 2]
      const oy2 = this.obstacleSegments[obsBase + 3]

      const obsMinX = Math.min(ox1, ox2), obsMaxX = Math.max(ox1, ox2)
      const obsMinY = Math.min(oy1, oy2), obsMaxY = Math.max(oy1, oy2)
      if (bi.maxX + preferredSpacing < obsMinX || obsMaxX + preferredSpacing < bi.minX ||
          bi.maxY + preferredSpacing < obsMinY || obsMaxY + preferredSpacing < bi.minY) continue

      for (let a = 0; a < OPT_SAMPLES; a++) {
        const a1x = pi[a * 2], a1y = pi[a * 2 + 1]
        const a2x = pi[(a + 1) * 2], a2y = pi[(a + 1) * 2 + 1]

        const distSq = segmentDistSq(a1x, a1y, a2x, a2y, ox1, oy1, ox2, oy2)
        if (distSq < spacingSq) {
          const dist = Math.sqrt(distSq)
          cost += (preferredSpacing - dist) ** 2
          if (distSq < 1e-18) cost += 20 * spacingSq
        }
      }
    }

    return cost
  }

  private optimizeStep() {
    const { bounds } = this.problem
    const { minX, maxX, minY, maxY } = bounds

    // Adaptive step size
    const progress = this.optimizationStep / this.maxOptimizationSteps
    const baseStep = 3.5 * (1 - progress) + 0.5
    const SQRT1_2 = Math.SQRT1_2

    // Sort traces by cost (worst first)
    const traceCosts: { idx: number; cost: number }[] = []
    for (let i = 0; i < this.traces.length; i++) {
      traceCosts.push({ idx: i, cost: this.computeCostForTrace(i) })
    }
    traceCosts.sort((a, b) => b.cost - a.cost)

    for (const { idx: i, cost: currentCost } of traceCosts) {
      if (currentCost === 0) continue

      const trace = this.traces[i]
      const steps = [baseStep, baseStep * 1.5, baseStep * 0.5]

      for (const step of steps) {
        const directions = [
          { dx: step, dy: 0 },
          { dx: -step, dy: 0 },
          { dx: 0, dy: step },
          { dx: 0, dy: -step },
          { dx: step * SQRT1_2, dy: step * SQRT1_2 },
          { dx: -step * SQRT1_2, dy: -step * SQRT1_2 },
          { dx: step * SQRT1_2, dy: -step * SQRT1_2 },
          { dx: -step * SQRT1_2, dy: step * SQRT1_2 },
        ]

        let bestCost = this.computeCostForTrace(i)
        let bestCtrl1x = trace.ctrl1.x
        let bestCtrl1y = trace.ctrl1.y
        let bestCtrl2x = trace.ctrl2.x
        let bestCtrl2y = trace.ctrl2.y

        const origCtrl1x = trace.ctrl1.x
        const origCtrl1y = trace.ctrl1.y
        const origCtrl2x = trace.ctrl2.x
        const origCtrl2y = trace.ctrl2.y

        for (const dir of directions) {
          // Try ctrl1
          trace.ctrl1.x = Math.max(minX, Math.min(maxX, origCtrl1x + dir.dx))
          trace.ctrl1.y = Math.max(minY, Math.min(maxY, origCtrl1y + dir.dy))
          this.updateSingleTraceSample(i)
          const cost1 = this.computeCostForTrace(i)
          if (cost1 < bestCost) {
            bestCost = cost1
            bestCtrl1x = trace.ctrl1.x
            bestCtrl1y = trace.ctrl1.y
            bestCtrl2x = origCtrl2x
            bestCtrl2y = origCtrl2y
          }
          trace.ctrl1.x = origCtrl1x
          trace.ctrl1.y = origCtrl1y

          // Try ctrl2
          trace.ctrl2.x = Math.max(minX, Math.min(maxX, origCtrl2x + dir.dx))
          trace.ctrl2.y = Math.max(minY, Math.min(maxY, origCtrl2y + dir.dy))
          this.updateSingleTraceSample(i)
          const cost2 = this.computeCostForTrace(i)
          if (cost2 < bestCost) {
            bestCost = cost2
            bestCtrl1x = origCtrl1x
            bestCtrl1y = origCtrl1y
            bestCtrl2x = trace.ctrl2.x
            bestCtrl2y = trace.ctrl2.y
          }
          trace.ctrl2.x = origCtrl2x
          trace.ctrl2.y = origCtrl2y

          // Try both together
          trace.ctrl1.x = Math.max(minX, Math.min(maxX, origCtrl1x + dir.dx))
          trace.ctrl1.y = Math.max(minY, Math.min(maxY, origCtrl1y + dir.dy))
          trace.ctrl2.x = Math.max(minX, Math.min(maxX, origCtrl2x + dir.dx))
          trace.ctrl2.y = Math.max(minY, Math.min(maxY, origCtrl2y + dir.dy))
          this.updateSingleTraceSample(i)
          const cost3 = this.computeCostForTrace(i)
          if (cost3 < bestCost) {
            bestCost = cost3
            bestCtrl1x = trace.ctrl1.x
            bestCtrl1y = trace.ctrl1.y
            bestCtrl2x = trace.ctrl2.x
            bestCtrl2y = trace.ctrl2.y
          }
          trace.ctrl1.x = origCtrl1x
          trace.ctrl1.y = origCtrl1y
          trace.ctrl2.x = origCtrl2x
          trace.ctrl2.y = origCtrl2y
        }

        trace.ctrl1.x = bestCtrl1x
        trace.ctrl1.y = bestCtrl1y
        trace.ctrl2.x = bestCtrl2x
        trace.ctrl2.y = bestCtrl2y
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
      points: sampleCubicBezier(trace.waypointPair.start, trace.ctrl1, trace.ctrl2, trace.waypointPair.end, OUTPUT_SAMPLES),
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

      const currentCost = this.computeTotalCost()
      if (currentCost === 0) {
        this.optimizationStep = this.maxOptimizationSteps
      } else if (currentCost >= this.lastCost * 0.99) {
        this.stagnantSteps++
        if (this.stagnantSteps > 15) {
          this.optimizationStep = this.maxOptimizationSteps
        }
      } else {
        this.stagnantSteps = 0
      }
      this.lastCost = currentCost
    }

    if (this.optimizationStep >= this.maxOptimizationSteps) {
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
