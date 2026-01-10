import { CurvyTraceSolver } from "../../lib/CurvyTraceSolver"
import { generateRandomProblem } from "../../lib/problem-generator"
import { scoreOutputCost } from "../../lib/scoreOutputCost"

const NUM_PROBLEMS = 25
const MIN_WAYPOINT_PAIRS = 2
const MAX_WAYPOINT_PAIRS = 12

function runBenchmark() {
  const scores: number[] = []
  const startTime = performance.now()

  console.log(`Running benchmark with ${NUM_PROBLEMS} problems...`)
  console.log(
    `Waypoint pairs range: ${MIN_WAYPOINT_PAIRS} to ${MAX_WAYPOINT_PAIRS}\n`,
  )

  for (let i = 0; i < NUM_PROBLEMS; i++) {
    const numWaypointPairs =
      MIN_WAYPOINT_PAIRS +
      Math.floor(Math.random() * (MAX_WAYPOINT_PAIRS - MIN_WAYPOINT_PAIRS + 1))

    const problem = generateRandomProblem({
      randomSeed: i,
      numWaypointPairs,
      numObstacles: 2,
      minSpacing: 5,
    })

    const solver = new CurvyTraceSolver(problem)

    // Run solver until complete (with max iterations to prevent infinite loops)
    const MAX_ITERATIONS = 1000
    for (let iter = 0; iter < MAX_ITERATIONS && !solver.solved; iter++) {
      solver.step()
    }

    const score = scoreOutputCost({
      problem,
      outputTraces: solver.outputTraces,
    })

    scores.push(score)

    // Progress indicator for each problem
    process.stdout.write(
      `\r  Progress: ${i + 1}/${NUM_PROBLEMS} problems completed`,
    )
  }
  console.log() // New line after progress

  const endTime = performance.now()
  const totalTimeMs = endTime - startTime

  // Calculate statistics
  const sortedScores = [...scores].sort((a, b) => a - b)
  const averageScore = scores.reduce((a, b) => a + b, 0) / scores.length

  // P95 worst score (95th percentile - higher scores are worse)
  const p95Index = Math.ceil(scores.length * 0.95) - 1
  const p95WorstScore = sortedScores[p95Index]

  // Additional stats
  const minScore = sortedScores[0]
  const maxScore = sortedScores[sortedScores.length - 1]
  const medianScore = sortedScores[Math.floor(scores.length / 2)]

  console.log("\n" + "=".repeat(50))
  console.log("BENCHMARK RESULTS")
  console.log("=".repeat(50))
  console.log(`Total problems:     ${NUM_PROBLEMS}`)
  console.log(`Total time:         ${(totalTimeMs / 1000).toFixed(2)}s`)
  console.log(
    `Avg time/problem:   ${(totalTimeMs / NUM_PROBLEMS).toFixed(2)}ms`,
  )
  console.log("-".repeat(50))
  console.log(`Average score:      ${averageScore.toFixed(2)}`)
  console.log(`P95 worst score:    ${p95WorstScore.toFixed(2)}`)
  console.log(`Median score:       ${medianScore.toFixed(2)}`)
  console.log(`Min score (best):   ${minScore.toFixed(2)}`)
  console.log(`Max score (worst):  ${maxScore.toFixed(2)}`)
  console.log("=".repeat(50))
}

runBenchmark()
