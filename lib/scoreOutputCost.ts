import type { CurvyTraceProblem, OutputTrace } from "./types"
import type { Point } from "@tscircuit/math-utils"
import { segmentToSegmentMinDistance } from "@tscircuit/math-utils"
import { sampleTraceIntoSegments } from "./sampleTraceIntoSegments"

/**
 * Returns a cost for a solution, lower is better output
 *
 * The cost is computed by sampling the output traces into samplesPerTrace
 * segments, then computing the minimum segment to segment distance to all
 * other-network segments in the output and obstacles. Beyond
 * problem.preferredSpacing the cost is 0. The max segment-to-segment cost
 * is problem.preferredSpacing**2
 *
 * If a segment intersects any segment from a different net, an additional
 * penalty of samplesPerTrace * preferredSpacing**2 is added.
 */
export const scoreOutputCost = ({
  problem,
  outputTraces,
  samplesPerTrace = 20,
}: {
  problem: CurvyTraceProblem
  outputTraces: OutputTrace[]
  samplesPerTrace?: number
}): number => {
  const { preferredSpacing, obstacles } = problem

  // Collect all trace segments
  const allTraceSegments: { segment: [Point, Point]; networkId?: string }[] = []
  for (const trace of outputTraces) {
    const segments = sampleTraceIntoSegments(
      trace.points,
      samplesPerTrace,
      trace.networkId,
    )
    allTraceSegments.push(...segments)
  }

  // Collect all obstacle segments
  const allObstacleSegments: { segment: [Point, Point]; networkId?: string }[] =
    []
  for (const obstacle of obstacles) {
    if (obstacle.outerSegments) {
      for (const seg of obstacle.outerSegments) {
        allObstacleSegments.push({
          segment: seg,
          networkId: obstacle.networkId,
        })
      }
    }
  }

  // Compute total cost
  let totalCost = 0

  for (let i = 0; i < allTraceSegments.length; i++) {
    const seg1 = allTraceSegments[i]

    // Check against other trace segments (different network)
    for (let j = i + 1; j < allTraceSegments.length; j++) {
      const seg2 = allTraceSegments[j]

      // Skip if same network (traces in same network are allowed to intersect)
      if (
        seg1.networkId &&
        seg2.networkId &&
        seg1.networkId === seg2.networkId
      ) {
        continue
      }

      const dist = segmentToSegmentMinDistance(
        seg1.segment[0],
        seg1.segment[1],
        seg2.segment[0],
        seg2.segment[1],
      )
      if (dist < preferredSpacing) {
        totalCost += (preferredSpacing - dist) ** 2
      }
      // Add cross-net intersection penalty
      if (dist < 1e-9) {
        totalCost += samplesPerTrace * preferredSpacing ** 2
      }
    }

    // Check against obstacle segments (different network)
    for (const obsSeg of allObstacleSegments) {
      // Skip if same network
      if (
        seg1.networkId &&
        obsSeg.networkId &&
        seg1.networkId === obsSeg.networkId
      ) {
        continue
      }

      const dist = segmentToSegmentMinDistance(
        seg1.segment[0],
        seg1.segment[1],
        obsSeg.segment[0],
        obsSeg.segment[1],
      )
      if (dist < preferredSpacing) {
        totalCost += (preferredSpacing - dist) ** 2
      }
      // Add cross-net intersection penalty
      if (dist < 1e-9) {
        totalCost += samplesPerTrace * preferredSpacing ** 2
      }
    }
  }

  return totalCost
}
