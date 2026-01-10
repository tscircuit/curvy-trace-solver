import { BaseSolver } from "@tscircuit/solver-utils"
import type { CurvyTraceProblem, OutputTrace } from "./types"
import type { GraphicsObject } from "graphics-debug"
import { getBoundsCenter } from "./geometry"
import { visualizeCurvyTraceProblem } from "./visualization-utils"
import { getObstacleOuterSegments } from "./geometry/getObstacleOuterSegments"
import type { Point, Bounds } from "@tscircuit/math-utils"

type TraceState = {
  waypointPair: CurvyTraceProblem["waypointPairs"][number]
  control1: Point
  control2: Point
  baseControl1: Point
  baseControl2: Point
}

const clampPointToBounds = (point: Point, bounds: Bounds) => {
  return {
    x: Math.min(bounds.maxX, Math.max(bounds.minX, point.x)),
    y: Math.min(bounds.maxY, Math.max(bounds.minY, point.y)),
  }
}

const cubicBezierPoint = (
  p0: Point,
  p1: Point,
  p2: Point,
  p3: Point,
  t: number,
): Point => {
  const u = 1 - t
  const tt = t * t
  const uu = u * u
  const uuu = uu * u
  const ttt = tt * t

  return {
    x: uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x,
    y: uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y,
  }
}

const sampleCubicBezier = (
  p0: Point,
  p1: Point,
  p2: Point,
  p3: Point,
  samples: number,
): Point[] => {
  const points: Point[] = []
  for (let i = 0; i <= samples; i++) {
    const t = i / samples
    points.push(cubicBezierPoint(p0, p1, p2, p3, t))
  }
  return points
}

const normalize = (dx: number, dy: number) => {
  const len = Math.hypot(dx, dy)
  if (len === 0) return { x: 0, y: 0 }
  return { x: dx / len, y: dy / len }
}

export class CurvyTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []
  private traceStates: TraceState[] = []
  private iteration = 0
  private readonly maxIterations = 120

  constructor(public problem: CurvyTraceProblem) {
    super()

    // Mutate the obstacles to compute the outer segments
    for (const obstacle of this.problem.obstacles) {
      obstacle.outerSegments = getObstacleOuterSegments(obstacle)
    }
  }

  override getConstructorParams() {
    return this.problem
  }

  override _step() {
    // We perform one step of the algorithm here, if the algorithm is multi-step
    // we perform small steps so that a human can see the progress- this method
    // will automatically be called in a loop until this.solved=true
    if (this.traceStates.length === 0) {
      const { bounds, preferredSpacing } = this.problem
      const center = getBoundsCenter(bounds)

      this.traceStates = this.problem.waypointPairs.map(
        (waypointPair, index): TraceState => {
          const dx = waypointPair.end.x - waypointPair.start.x
          const dy = waypointPair.end.y - waypointPair.start.y
          const baseControl1 = {
            x: waypointPair.start.x + dx / 3,
            y: waypointPair.start.y + dy / 3,
          }
          const baseControl2 = {
            x: waypointPair.start.x + (2 * dx) / 3,
            y: waypointPair.start.y + (2 * dy) / 3,
          }

          const { x: nx, y: ny } = normalize(-dy, dx)
          const mid = {
            x: (waypointPair.start.x + waypointPair.end.x) / 2,
            y: (waypointPair.start.y + waypointPair.end.y) / 2,
          }
          const toCenter = { x: center.x - mid.x, y: center.y - mid.y }
          const dot = nx * toCenter.x + ny * toCenter.y
          const direction = dot > 0 ? -1 : 1
          const offset =
            preferredSpacing * 0.45 + (index % 3) * preferredSpacing * 0.1
          const offsetX = nx * offset * direction
          const offsetY = ny * offset * direction

          return {
            waypointPair,
            baseControl1,
            baseControl2,
            control1: clampPointToBounds(
              {
                x: baseControl1.x + offsetX,
                y: baseControl1.y + offsetY,
              },
              bounds,
            ),
            control2: clampPointToBounds(
              {
                x: baseControl2.x + offsetX,
                y: baseControl2.y + offsetY,
              },
              bounds,
            ),
          }
        },
      )
    }

    const { preferredSpacing, bounds } = this.problem
    const repulsionRadius = preferredSpacing * 1.2
    const stepScale = preferredSpacing * 0.12
    const springStrength = 0.08

    const sampleTValues = [0, 0.2, 0.4, 0.6, 0.8, 1]
    const traceSamples = this.traceStates.map((state) => {
      const { start, end } = state.waypointPair
      return sampleTValues.map((t) =>
        cubicBezierPoint(start, state.control1, state.control2, end, t),
      )
    })

    const nextStates: TraceState[] = this.traceStates.map((state, index) => {
      const { start, end } = state.waypointPair

      const controlTargets = [
        { t: 0.33, controlKey: "control1" as const },
        { t: 0.67, controlKey: "control2" as const },
      ]

      const updated = { ...state }

      for (const target of controlTargets) {
        const currentPoint = cubicBezierPoint(
          start,
          updated.control1,
          updated.control2,
          end,
          target.t,
        )
        let forceX = 0
        let forceY = 0

        for (let otherIndex = 0; otherIndex < traceSamples.length; otherIndex++) {
          if (otherIndex === index) continue
          for (const sample of traceSamples[otherIndex]) {
            const dx = currentPoint.x - sample.x
            const dy = currentPoint.y - sample.y
            const dist = Math.hypot(dx, dy)
            if (dist > 0 && dist < repulsionRadius) {
              const strength = (repulsionRadius - dist) / repulsionRadius
              const norm = normalize(dx, dy)
              forceX += norm.x * strength
              forceY += norm.y * strength
            }
          }
        }

        if (forceX !== 0 || forceY !== 0) {
          updated[target.controlKey] = clampPointToBounds(
            {
              x: updated[target.controlKey].x + forceX * stepScale,
              y: updated[target.controlKey].y + forceY * stepScale,
            },
            bounds,
          )
        }
      }

      updated.control1 = clampPointToBounds(
        {
          x:
            updated.control1.x +
            (updated.baseControl1.x - updated.control1.x) * springStrength,
          y:
            updated.control1.y +
            (updated.baseControl1.y - updated.control1.y) * springStrength,
        },
        bounds,
      )
      updated.control2 = clampPointToBounds(
        {
          x:
            updated.control2.x +
            (updated.baseControl2.x - updated.control2.x) * springStrength,
          y:
            updated.control2.y +
            (updated.baseControl2.y - updated.control2.y) * springStrength,
        },
        bounds,
      )

      return updated
    })

    this.traceStates = nextStates

    this.outputTraces = this.traceStates.map((state) => {
      const { start, end } = state.waypointPair
      const points = sampleCubicBezier(
        start,
        state.control1,
        state.control2,
        end,
        16,
      )
      return {
        waypointPair: state.waypointPair,
        points,
        networkId: state.waypointPair.networkId,
      }
    })

    this.iteration += 1
    if (this.iteration >= this.maxIterations) {
      this.solved = true
    }
  }

  override visualize(): GraphicsObject {
    return visualizeCurvyTraceProblem(this.problem, this.outputTraces)
  }
}
