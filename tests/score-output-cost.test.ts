import { expect, test } from "bun:test"
import { scoreOutputCost } from "../lib/scoreOutputCost"
import type { CurvyTraceProblem, OutputTrace } from "../lib/types"

test("uses obstacle spacing when scoring trace obstacle clearance", () => {
  const problem: CurvyTraceProblem = {
    bounds: { minX: 0, minY: 0, maxX: 10, maxY: 10 },
    waypointPairs: [],
    preferredTraceToTraceSpacing: 2,
    preferredObstacleToTraceSpacing: 0.5,
    obstacles: [
      {
        minX: 4,
        maxX: 6,
        minY: 1,
        maxY: 2,
        center: { x: 5, y: 1.5 },
        networkId: "obstacle-net",
        outerSegments: [
          [
            { x: 4, y: 1 },
            { x: 6, y: 1 },
          ],
        ],
      },
    ],
  }
  const outputTraces: OutputTrace[] = [
    {
      waypointPair: {
        start: { x: 0, y: 0 },
        end: { x: 10, y: 0 },
        networkId: "trace-net",
      },
      points: [
        { x: 0, y: 0 },
        { x: 10, y: 0 },
      ],
      networkId: "trace-net",
    },
  ]

  expect(scoreOutputCost({ problem, outputTraces, samplesPerTrace: 4 })).toBe(0)
})
