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
  t1: number // Perimeter position of start
  t2: number // Perimeter position of end
}

// Get perimeter position (0 to perimeter length) for a point on the boundary
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

// Get point on perimeter at position t (0 to perimeter length)
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

// Sample a cubic bezier curve
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

// Point to segment distance
function pointToSegmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x
  const dy = b.y - a.y
  const lengthSq = dx * dx + dy * dy
  if (lengthSq === 0) return Math.hypot(p.x - a.x, p.y - a.y)
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSq))
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy))
}

// Segment to segment distance
function segmentToSegmentDistance(a1: Point, a2: Point, b1: Point, b2: Point): number {
  const cross = (o: Point, a: Point, b: Point) =>
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
  const d1 = cross(b1, b2, a1)
  const d2 = cross(b1, b2, a2)
  const d3 = cross(a1, a2, b1)
  const d4 = cross(a1, a2, b2)

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return 0
  }

  return Math.min(
    pointToSegmentDistance(a1, b1, b2),
    pointToSegmentDistance(a2, b1, b2),
    pointToSegmentDistance(b1, a1, a2),
    pointToSegmentDistance(b2, a1, a2),
  )
}

// Check if chord A contains chord B (B is nested inside A)
function chordContains(
  aT1: number,
  aT2: number,
  bT1: number,
  bT2: number,
  perimeter: number,
): boolean {
  // Normalize to [0, perimeter)
  const normalize = (t: number) => ((t % perimeter) + perimeter) % perimeter
  const a1 = normalize(aT1)
  const a2 = normalize(aT2)
  const b1 = normalize(bT1)
  const b2 = normalize(bT2)

  // Make a1 < a2
  const [aMin, aMax] = a1 < a2 ? [a1, a2] : [a2, a1]
  // Both b points should be between aMin and aMax
  return b1 > aMin && b1 < aMax && b2 > aMin && b2 < aMax
}

// Get bounds center
function getBoundsCenter(bounds: Bounds): Point {
  return { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 }
}

export class CurvyTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traces: TraceWithControlPoints[] = []
  private optimizationStep = 0
  private readonly maxOptimizationSteps = 100
  private sampledTraces: Map<number, Point[]> = new Map()

  constructor(public problem: CurvyTraceProblem) {
    super()
    for (const obstacle of this.problem.obstacles) {
      obstacle.outerSegments = getObstacleOuterSegments(obstacle)
    }
  }

  override getConstructorParams() {
    return this.problem
  }

  private initializeTraces() {
    const { bounds, waypointPairs } = this.problem
    const { minX, maxX, minY, maxY } = bounds
    const W = maxX - minX
    const H = maxY - minY
    const perimeter = 2 * W + 2 * H
    const center = getBoundsCenter(bounds)

    // Compute perimeter positions for all waypoints
    const tracesWithT = waypointPairs.map((pair, idx) => {
      const t1 = getPerimeterT(pair.start, bounds)
      const t2 = getPerimeterT(pair.end, bounds)
      return { pair, t1, t2, idx }
    })

    // Compute nesting depth for each trace
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
      // Determine routing direction (shorter arc)
      let dt = t2 - t1
      if (dt > perimeter / 2) dt -= perimeter
      if (dt < -perimeter / 2) dt += perimeter

      // Control points at 1/3 and 2/3 along the arc
      const tCtrl1 = t1 + dt * 0.33
      const tCtrl2 = t1 + dt * 0.67

      const pPerim1 = getPerimeterPoint(tCtrl1, bounds)
      const pPerim2 = getPerimeterPoint(tCtrl2, bounds)

      // Linear interpolation points (straight line)
      const pLinear1 = {
        x: pair.start.x + (pair.end.x - pair.start.x) * 0.33,
        y: pair.start.y + (pair.end.y - pair.start.y) * 0.33,
      }
      const pLinear2 = {
        x: pair.start.x + (pair.end.x - pair.start.x) * 0.67,
        y: pair.start.y + (pair.end.y - pair.start.y) * 0.67,
      }

      // How far the midpoint is from the center
      const midPoint = { x: (pair.start.x + pair.end.x) / 2, y: (pair.start.y + pair.end.y) / 2 }
      const distToCenter = Math.hypot(midPoint.x - center.x, midPoint.y - center.y)
      const maxDist = Math.hypot(W / 2, H / 2)
      const spatialDepth = 1 - distToCenter / maxDist

      const depth = nestingDepth.get(idx) || 0
      const normalizedDepth = depth / maxDepth

      // Pull toward perimeter based on depth
      const basePull = 0.3 + spatialDepth * 0.4
      const pullAmount = Math.max(0.05, basePull - normalizedDepth * 0.2)

      const ctrl1 = {
        x: pLinear1.x * (1 - pullAmount) + pPerim1.x * pullAmount,
        y: pLinear1.y * (1 - pullAmount) + pPerim1.y * pullAmount,
      }
      const ctrl2 = {
        x: pLinear2.x * (1 - pullAmount) + pPerim2.x * pullAmount,
        y: pLinear2.y * (1 - pullAmount) + pPerim2.y * pullAmount,
      }

      return { waypointPair: pair, ctrl1, ctrl2, networkId: pair.networkId, t1, t2 }
    })

    this.updateSampledTraces()
  }

  private updateSampledTraces() {
    this.sampledTraces.clear()
    for (let i = 0; i < this.traces.length; i++) {
      this.sampledTraces.set(i, this.sampleTrace(this.traces[i], 12))
    }
  }

  private sampleTrace(trace: TraceWithControlPoints, numSamples: number): Point[] {
    return sampleCubicBezier(trace.waypointPair.start, trace.ctrl1, trace.ctrl2, trace.waypointPair.end, numSamples)
  }

  private computeTotalCost(): number {
    const { preferredSpacing } = this.problem
    let cost = 0

    for (let i = 0; i < this.traces.length; i++) {
      const trace = this.traces[i]
      const points = this.sampledTraces.get(i)!

      for (let j = i + 1; j < this.traces.length; j++) {
        const other = this.traces[j]
        if (trace.networkId && other.networkId && trace.networkId === other.networkId) continue

        const otherPoints = this.sampledTraces.get(j)!

        for (let a = 0; a < points.length - 1; a++) {
          for (let b = 0; b < otherPoints.length - 1; b++) {
            const dist = segmentToSegmentDistance(points[a], points[a + 1], otherPoints[b], otherPoints[b + 1])
            if (dist < preferredSpacing) {
              cost += (preferredSpacing - dist) ** 2
            }
            if (dist < 1e-9) {
              cost += 20 * preferredSpacing ** 2
            }
          }
        }
      }
    }

    return cost
  }

  private computeCostForTrace(traceIdx: number): number {
    const { preferredSpacing } = this.problem
    const trace = this.traces[traceIdx]
    const points = this.sampledTraces.get(traceIdx)!
    let cost = 0

    for (let j = 0; j < this.traces.length; j++) {
      if (j === traceIdx) continue
      const other = this.traces[j]
      if (trace.networkId && other.networkId && trace.networkId === other.networkId) continue

      const otherPoints = this.sampledTraces.get(j)!

      for (let a = 0; a < points.length - 1; a++) {
        for (let b = 0; b < otherPoints.length - 1; b++) {
          const dist = segmentToSegmentDistance(points[a], points[a + 1], otherPoints[b], otherPoints[b + 1])
          if (dist < preferredSpacing) {
            cost += (preferredSpacing - dist) ** 2
          }
          if (dist < 1e-9) {
            cost += 20 * preferredSpacing ** 2
          }
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

    // Process traces in order of cost (worst first)
    const traceCosts = this.traces.map((_, i) => ({ idx: i, cost: this.computeCostForTrace(i) }))
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
        let bestCtrl1 = { ...trace.ctrl1 }
        let bestCtrl2 = { ...trace.ctrl2 }

        const origCtrl1 = { ...trace.ctrl1 }
        const origCtrl2 = { ...trace.ctrl2 }

        for (const dir of directions) {
          // Try ctrl1
          trace.ctrl1 = {
            x: Math.max(minX, Math.min(maxX, origCtrl1.x + dir.dx)),
            y: Math.max(minY, Math.min(maxY, origCtrl1.y + dir.dy)),
          }
          this.sampledTraces.set(i, this.sampleTrace(trace, 12))
          const cost1 = this.computeCostForTrace(i)
          if (cost1 < bestCost) {
            bestCost = cost1
            bestCtrl1 = { ...trace.ctrl1 }
          }
          trace.ctrl1 = origCtrl1

          // Try ctrl2
          trace.ctrl2 = {
            x: Math.max(minX, Math.min(maxX, origCtrl2.x + dir.dx)),
            y: Math.max(minY, Math.min(maxY, origCtrl2.y + dir.dy)),
          }
          this.sampledTraces.set(i, this.sampleTrace(trace, 12))
          const cost2 = this.computeCostForTrace(i)
          if (cost2 < bestCost) {
            bestCost = cost2
            bestCtrl2 = { ...trace.ctrl2 }
          }
          trace.ctrl2 = origCtrl2

          // Try both together
          trace.ctrl1 = {
            x: Math.max(minX, Math.min(maxX, origCtrl1.x + dir.dx)),
            y: Math.max(minY, Math.min(maxY, origCtrl1.y + dir.dy)),
          }
          trace.ctrl2 = {
            x: Math.max(minX, Math.min(maxX, origCtrl2.x + dir.dx)),
            y: Math.max(minY, Math.min(maxY, origCtrl2.y + dir.dy)),
          }
          this.sampledTraces.set(i, this.sampleTrace(trace, 12))
          const cost3 = this.computeCostForTrace(i)
          if (cost3 < bestCost) {
            bestCost = cost3
            bestCtrl1 = { ...trace.ctrl1 }
            bestCtrl2 = { ...trace.ctrl2 }
          }
          trace.ctrl1 = origCtrl1
          trace.ctrl2 = origCtrl2
        }

        trace.ctrl1 = bestCtrl1
        trace.ctrl2 = bestCtrl2
        this.sampledTraces.set(i, this.sampleTrace(trace, 12))

        if (bestCost < currentCost * 0.9) break // Found significant improvement
      }
    }
  }

  private buildOutputTraces() {
    this.outputTraces = this.traces.map((trace) => ({
      waypointPair: trace.waypointPair,
      points: this.sampleTrace(trace, 20),
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

      // Check for early termination
      const currentCost = this.computeTotalCost()
      if (currentCost === 0) {
        this.optimizationStep = this.maxOptimizationSteps
      } else if (currentCost >= this.lastCost * 0.99) {
        this.stagnantSteps++
        if (this.stagnantSteps > 15) {
          // No progress for 15 steps, terminate early
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

  private lastCost = Infinity
  private stagnantSteps = 0

  override visualize(): GraphicsObject {
    if (this.traces.length > 0) {
      this.buildOutputTraces()
    }
    return visualizeCurvyTraceProblem(this.problem, this.outputTraces)
  }
}
