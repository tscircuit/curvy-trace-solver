import { expect, test } from "bun:test"
import { getSvgFromGraphicsObject } from "graphics-debug"
import { CurvyTraceSolver } from "lib/CurvyTraceSolver"
import problem from "./repro01-input.json"
import type { CurvyTraceProblem } from "lib/types"

test("repro01", () => {
  const solver = new CurvyTraceSolver(problem as CurvyTraceProblem)
  solver.solve()
  const svg = getSvgFromGraphicsObject(solver.visualize())
  expect(svg).toMatchSvgSnapshot(import.meta.path)
})
