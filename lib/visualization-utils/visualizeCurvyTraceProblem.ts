import type { GraphicsObject } from "graphics-debug"
import type { CurvyTraceProblem, OutputTrace } from "../types"
import { getColorForNetworkId } from "./index"

export const visualizeCurvyTraceProblem = (
  problem: CurvyTraceProblem,
  outputTraces: OutputTrace[] = [],
): GraphicsObject => {
  const graphics = {
    arrows: [],
    circles: [],
    lines: [],
    rects: [],
    coordinateSystem: "cartesian",
    points: [],
    texts: [],
    title: "Curvy Trace Problem",
  } as Required<GraphicsObject>

  // Draw bounds
  graphics.lines.push({
    points: [
      { x: problem.bounds.minX, y: problem.bounds.minY },
      { x: problem.bounds.maxX, y: problem.bounds.minY },
      { x: problem.bounds.maxX, y: problem.bounds.maxY },
      { x: problem.bounds.minX, y: problem.bounds.maxY },
      { x: problem.bounds.minX, y: problem.bounds.minY },
    ],
    strokeColor: "rgba(0, 0, 0, 0.1)",
  })

  // Draw waypoint pairs
  for (const waypointPair of problem.waypointPairs) {
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

  // Draw obstacles
  for (const obstacle of problem.obstacles) {
    graphics.rects.push({
      center: obstacle.center,
      width: obstacle.maxX - obstacle.minX,
      height: obstacle.maxY - obstacle.minY,
      fill: "rgba(128, 128, 128, 0.3)",
      stroke: "rgba(128, 128, 128, 0.8)",
    })
  }

  // Draw output traces
  for (const trace of outputTraces) {
    graphics.lines.push({
      points: trace.points,
      strokeColor: getColorForNetworkId(trace.networkId),
    })
  }

  return graphics
}
