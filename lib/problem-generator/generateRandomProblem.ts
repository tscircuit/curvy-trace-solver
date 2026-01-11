import type { Bounds } from "@tscircuit/math-utils"
import type { CurvyTraceProblem, Obstacle, WaypointPair } from "lib/types"
import { perimeterT } from "./countChordCrossings"
import { createRng } from "./createRng"
import { randomBoundaryPoint } from "./randomBoundaryPoint"
import { wouldCrossAny } from "./wouldCrossAny"

type Side = "top" | "bottom" | "left" | "right"

/**
 * Check if two obstacles overlap.
 */
const obstaclesOverlap = (a: Obstacle, b: Obstacle): boolean => {
  return !(
    a.maxX <= b.minX ||
    a.minX >= b.maxX ||
    a.maxY <= b.minY ||
    a.minY >= b.maxY
  )
}

/**
 * Generate an obstacle that is outside the bounds but touching a specific side.
 * The obstacle takes up at least 10% of the side it's touching.
 */
const generateObstacleOnSide = (
  rng: () => number,
  bounds: Bounds,
  side: Side,
): Obstacle => {
  const { minX, maxX, minY, maxY } = bounds
  const W = maxX - minX
  const H = maxY - minY

  // Obstacle depth (how far it extends outside the bounds)
  const minDepth = Math.min(W, H) * 0.1
  const maxDepth = Math.min(W, H) * 0.3
  const depth = minDepth + rng() * (maxDepth - minDepth)

  let obstacleMinX: number
  let obstacleMaxX: number
  let obstacleMinY: number
  let obstacleMaxY: number

  if (side === "top" || side === "bottom") {
    // Obstacle spans horizontally along top or bottom
    const minSpan = W * 0.1 // At least 10% of the side
    const maxSpan = W * 0.4 // Up to 40% of the side
    const span = minSpan + rng() * (maxSpan - minSpan)

    // Random position along the side (ensuring it fits within bounds)
    const startX = minX + rng() * (W - span)
    obstacleMinX = startX
    obstacleMaxX = startX + span

    if (side === "top") {
      obstacleMinY = maxY
      obstacleMaxY = maxY + depth
    } else {
      obstacleMinY = minY - depth
      obstacleMaxY = minY
    }
  } else {
    // Obstacle spans vertically along left or right
    const minSpan = H * 0.1 // At least 10% of the side
    const maxSpan = H * 0.4 // Up to 40% of the side
    const span = minSpan + rng() * (maxSpan - minSpan)

    // Random position along the side (ensuring it fits within bounds)
    const startY = minY + rng() * (H - span)
    obstacleMinY = startY
    obstacleMaxY = startY + span

    if (side === "right") {
      obstacleMinX = maxX
      obstacleMaxX = maxX + depth
    } else {
      obstacleMinX = minX - depth
      obstacleMaxX = minX
    }
  }

  return {
    minX: obstacleMinX,
    maxX: obstacleMaxX,
    minY: obstacleMinY,
    maxY: obstacleMaxY,
    center: {
      x: (obstacleMinX + obstacleMaxX) / 2,
      y: (obstacleMinY + obstacleMaxY) / 2,
    },
  }
}

export const generateRandomProblem = (opts: {
  numWaypointPairs: number
  numObstacles: number
  randomSeed: number
  bounds?: Bounds
  preferredTraceToTraceSpacing?: number
  preferredTraceToObstacleSpacing?: number
  /**
   * Minimum spacing between a waypoint point and any other waypoint point.
   */
  minSpacing?: number
}): CurvyTraceProblem => {
  const rng = createRng(opts.randomSeed)

  const bounds = opts.bounds ?? { minX: 0, maxX: 100, minY: 0, maxY: 100 }
  const { minX: xmin, maxX: xmax, minY: ymin, maxY: ymax } = bounds
  const W = xmax - xmin
  const H = ymax - ymin
  const perimeter = 2 * W + 2 * H

  const waypointPairs: WaypointPair[] = []
  const chords: [number, number][] = []

  const MAX_ATTEMPTS = 1000

  for (let i = 0; i < opts.numWaypointPairs; i++) {
    let attempts = 0
    let start: { x: number; y: number } | null = null
    let end: { x: number; y: number } | null = null
    let newChord: [number, number] | null = null

    while (attempts < MAX_ATTEMPTS) {
      // Generate two random boundary points
      start = randomBoundaryPoint(rng, xmin, xmax, ymin, ymax)
      end = randomBoundaryPoint(rng, xmin, xmax, ymin, ymax)

      // Convert to perimeter t values using existing utility
      const t1 = perimeterT(start, xmin, xmax, ymin, ymax)
      const t2 = perimeterT(end, xmin, xmax, ymin, ymax)

      // Ensure the two points aren't too close together
      const minSeparation = perimeter * 0.05
      const dist = Math.min(Math.abs(t1 - t2), perimeter - Math.abs(t1 - t2))
      if (dist < minSeparation) {
        attempts++
        continue
      }

      newChord = [t1, t2]

      // Check minSpacing constraint against all existing waypoint points
      if (opts.minSpacing !== undefined) {
        let tooClose = false
        for (const pair of waypointPairs) {
          const dStart1 = Math.hypot(
            start.x - pair.start.x,
            start.y - pair.start.y,
          )
          const dStart2 = Math.hypot(start.x - pair.end.x, start.y - pair.end.y)
          const dEnd1 = Math.hypot(end.x - pair.start.x, end.y - pair.start.y)
          const dEnd2 = Math.hypot(end.x - pair.end.x, end.y - pair.end.y)
          if (
            dStart1 < opts.minSpacing ||
            dStart2 < opts.minSpacing ||
            dEnd1 < opts.minSpacing ||
            dEnd2 < opts.minSpacing
          ) {
            tooClose = true
            break
          }
        }
        if (tooClose) {
          attempts++
          continue
        }
      }

      // Check if this chord would cross any existing chords
      if (!wouldCrossAny(newChord, chords)) {
        break
      }

      newChord = null
      attempts++
    }

    if (newChord === null || start === null || end === null) {
      throw new Error(
        `Failed to generate non-crossing waypoint pair after ${MAX_ATTEMPTS} attempts. ` +
          `This may happen if too many waypoint pairs are requested.`,
      )
    }

    chords.push(newChord)
    waypointPairs.push({ start, end, networkId: `net${i}` })
  }

  // Generate obstacles outside the bounds but touching them
  const obstacles: Obstacle[] = []
  const sides: Side[] = ["top", "bottom", "left", "right"]

  for (let i = 0; i < opts.numObstacles; i++) {
    let attempts = 0
    let obstacle: Obstacle | null = null

    while (attempts < MAX_ATTEMPTS) {
      // Pick a random side for this obstacle
      const side = sides[Math.floor(rng() * sides.length)]
      const candidate = generateObstacleOnSide(rng, bounds, side)

      // Check if it overlaps with any existing obstacle
      const overlaps = obstacles.some((existing) =>
        obstaclesOverlap(candidate, existing),
      )

      if (!overlaps) {
        obstacle = candidate
        break
      }

      attempts++
    }

    if (obstacle === null) {
      // Could not place obstacle without overlap, skip it
      break
    }

    obstacles.push(obstacle)
  }

  return {
    bounds,
    waypointPairs,
    obstacles,
    preferredTraceToTraceSpacing: opts.preferredTraceToTraceSpacing ?? 10,
    preferredObstacleToTraceSpacing: opts.preferredTraceToObstacleSpacing ?? 5,
  }
}
