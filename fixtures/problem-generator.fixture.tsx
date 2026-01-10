import { useState, useMemo } from "react"
import { InteractiveGraphics } from "graphics-debug/react"
import { generateRandomProblem } from "../lib/problem-generator"
import { visualizeCurvyTraceProblem } from "../lib/visualization-utils"

export default () => {
  const [randomSeed, setRandomSeed] = useState(1)
  const [numWaypointPairs, setNumWaypointPairs] = useState(5)

  const graphics = useMemo(() => {
    try {
      const problem = generateRandomProblem({
        randomSeed,
        numWaypointPairs,
        numObstacles: 0,
      })
      return visualizeCurvyTraceProblem(problem)
    } catch (e) {
      return {
        coordinateSystem: "cartesian" as const,
        texts: [{ text: `Error: ${(e as Error).message}`, x: 50, y: 50 }],
      }
    }
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
      <InteractiveGraphics graphics={graphics} />
    </div>
  )
}
