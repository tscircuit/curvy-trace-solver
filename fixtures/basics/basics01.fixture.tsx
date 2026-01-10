import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import type { CurvyTraceProblem } from "../../lib/types"
import { CurvyTraceSolver } from "../../lib/CurvyTraceSolver"
import problem from "./basics01-input.json"
export default () => {
  return (
    <GenericSolverDebugger
      createSolver={() => new CurvyTraceSolver(problem as CurvyTraceProblem)}
    />
  )
}
