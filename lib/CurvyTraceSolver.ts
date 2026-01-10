import { BaseSolver } from "@tscircuit/solver-utils"
import type { CurvyTraceProblem, OutputTrace } from "./types"
import type { GraphicsObject } from "graphics-debug"
import { getBoundsCenter } from "./geometry"
import { visualizeCurvyTraceProblem } from "./visualization-utils"
import { getObstacleOuterSegments } from "./geometry/getObstacleOuterSegments"

export class CurvyTraceSolver extends BaseSolver {
  outputTraces: OutputTrace[] = []

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

    // ------------- REMOVE THIS -------------
    // naive implementation that just connects the waypoints with a straight line
    for (const waypointPair of this.problem.waypointPairs) {
      this.outputTraces.push({
        waypointPair,
        points: [waypointPair.start, waypointPair.end],
        networkId: waypointPair.networkId,
      })
    }
    // ------------- END REMOVE THIS -------------
  }

  override visualize(): GraphicsObject {
    return visualizeCurvyTraceProblem(this.problem, this.outputTraces)
  }
}
