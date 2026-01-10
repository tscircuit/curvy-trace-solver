import type { CurvyTraceProblem, OutputTrace } from "./types"
import { segmentToSegmentMinDistance } from "@tscircuit/math-utils"

/**
 * Returns a cost for a solution, lower is better output
 *
 * The cost is computed by sampling the output traces into samplesPerTrace
 * segments, then computing the minimum segment to segment distance to all
 * other-network segments in the output and obstacles. Beyond
 * problem.preferredSpacing the cost is 0. The max segment-to-segment cost
 * is problem.preferredSpacing**2
 */
export const scoreOutputCost = ({
  problem,
  outputTraces,
  samplesPerTrace = 20,
}: {
  problem: CurvyTraceProblem
  outputTraces: OutputTrace[]
  samplesPerTrace: number
}) => {}
