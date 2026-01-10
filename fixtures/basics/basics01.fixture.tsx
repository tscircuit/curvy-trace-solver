import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import type { CurvyTraceProblem } from "../../lib/types"
import { CurvyTraceSolver } from "../../lib/CurvyTraceSolver"

const problem: CurvyTraceProblem = {
  bounds: { minX: 0, minY: 0, maxX: 100, maxY: 100 },
  waypointPairs: [
    {
      start: { x: 0, y: 10 },
      end: { x: 100, y: 80 },
    },
    {
      start: { x: 0, y: 20 },
      end: { x: 100, y: 90 },
    },
  ],
  obstacles: [],
  preferredSpacing: 25,
}

export default () => {
  return (
    <GenericSolverDebugger createSolver={() => new CurvyTraceSolver(problem)} />
  )
}
