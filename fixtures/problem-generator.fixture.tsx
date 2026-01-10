import { useState, useMemo } from "react"
import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { generateRandomProblem } from "../lib/problem-generator"
import { CurvyTraceSolver } from "../lib/CurvyTraceSolver"
import type { CurvyTraceProblem } from "../lib/types"

export default () => {
  const [randomSeed, setRandomSeed] = useState(1)
  const [numWaypointPairs, setNumWaypointPairs] = useState(5)

  const problem = useMemo(() => {
    return generateRandomProblem({
      randomSeed,
      numWaypointPairs,
      numObstacles: 3,
      minSpacing: 5,
    })
  }, [randomSeed, numWaypointPairs])

  return (
    <div style={{ padding: 16 }}>
      <div style={{ marginBottom: 16, display: "flex", gap: 16 }}>
        <label>
          Random Seed:{" "}
          <input
            type="number"
            value={randomSeed}
            onChange={(e) => setRandomSeed(Number(e.target.value))}
            style={{ width: 80 }}
          />
        </label>
        <label>
          Waypoint Pairs:{" "}
          <input
            type="number"
            value={numWaypointPairs}
            onChange={(e) => setNumWaypointPairs(Number(e.target.value))}
            style={{ width: 80 }}
            min={1}
            max={20}
          />
        </label>
      </div>
      <GenericSolverDebugger
        key={`${randomSeed}-${numWaypointPairs}`}
        createSolver={() => new CurvyTraceSolver(problem as CurvyTraceProblem)}
      />
    </div>
  )
}
