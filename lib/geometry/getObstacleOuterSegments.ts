import type { Point } from "@tscircuit/math-utils"
import type { Obstacle } from "lib/types"

export const getObstacleOuterSegments = (obstacle: Obstacle) => {
  const segments: [Point, Point][] = [
    [
      { x: obstacle.minX, y: obstacle.minY },
      { x: obstacle.maxX, y: obstacle.minY },
    ],
    [
      { x: obstacle.maxX, y: obstacle.minY },
      { x: obstacle.maxX, y: obstacle.maxY },
    ],
    [
      { x: obstacle.maxX, y: obstacle.maxY },
      { x: obstacle.minX, y: obstacle.maxY },
    ],
    [
      { x: obstacle.minX, y: obstacle.maxY },
      { x: obstacle.minX, y: obstacle.minY },
    ],
  ]

  return segments
}
