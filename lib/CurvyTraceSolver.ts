import { BaseSolver } from "@tscircuit/solver-utils"
import type { CurvyTraceProblem } from "./types"
import type { GraphicsObject } from "graphics-debug"
import { getBoundsCenter } from "./geometry"
import { getColorForNetworkId } from "./visualization-utils"

export class CurvyTraceSolver extends BaseSolver {
  constructor(public problem: CurvyTraceProblem) {
    super()
  }

  override getConstructorParams() {
    return this.problem
  }

  override _step() {
    // We perform one step of the algorithm here, if the algorithm is multi-step
    // we perform small steps so that a human can see the progress- this method
    // will automatically be called in a loop until this.solved=true
  }

  override visualize(): GraphicsObject {
    const graphics = {
      arrows: [],
      circles: [],
      lines: [],
      rects: [],
      coordinateSystem: "cartesian",
      points: [],
      texts: [],
      title: "Curvy Trace Solver",
    } as Required<GraphicsObject>

    graphics.lines.push({
      points: [
        { x: this.problem.bounds.minX, y: this.problem.bounds.minY },
        { x: this.problem.bounds.maxX, y: this.problem.bounds.minY },
        { x: this.problem.bounds.maxX, y: this.problem.bounds.maxY },
        { x: this.problem.bounds.minX, y: this.problem.bounds.maxY },
        { x: this.problem.bounds.minX, y: this.problem.bounds.minY },
      ],
      strokeColor: "rgba(0, 0, 0, 0.1)",
    })

    for (const waypointPair of this.problem.waypointPairs) {
      graphics.points.push({
        ...waypointPair.start,
        label: `start ${waypointPair.networkId ?? ""}`,
        color: getColorForNetworkId(waypointPair.networkId),
      })
      graphics.points.push({
        ...waypointPair.end,
        label: `end ${waypointPair.networkId ?? ""}`,
        color: getColorForNetworkId(waypointPair.networkId),
      })
    }

    return graphics
  }
}
