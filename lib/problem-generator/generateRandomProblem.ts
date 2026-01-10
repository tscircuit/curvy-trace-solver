import type { Bounds } from "@tscircuit/math-utils"
import type { CurvyTraceProblem, WaypointPair } from "lib/types"
import { perimeterT } from "./countChordCrossings"
import { createRng } from "./createRng"
import { randomBoundaryPoint } from "./randomBoundaryPoint"
import { wouldCrossAny } from "./wouldCrossAny"

export const generateRandomProblem = (opts: {
  numWaypointPairs: number
  numObstacles: number
  randomSeed: number
  bounds?: Bounds
  preferredSpacing?: number
  /**
   * Minimum spacing between a waypoint point and any other waypoint point.
   */
  minSpacing?: number
}): CurvyTraceProblem => {
  if (opts.numObstacles > 0) throw new Error("Obstacles are not supported yet")

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

  return {
    bounds,
    waypointPairs,
    obstacles: [],
    preferredSpacing: opts.preferredSpacing ?? 10,
  }
}
