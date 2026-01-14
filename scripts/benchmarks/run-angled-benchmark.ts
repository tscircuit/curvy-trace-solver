import { AngledTraceSolver } from "../../lib/AngledTraceSolver"
import { generateRandomProblem } from "../../lib/problem-generator"
import { scoreOutputCost } from "../../lib/scoreOutputCost"

const NUM_PROBLEMS = 100
const MIN_WAYPOINT_PAIRS = 2
const MAX_WAYPOINT_PAIRS = 12
const BENCHMARK_SEED = 42

// Simple seeded PRNG (mulberry32)
function createSeededRandom(seed: number) {
  return () => {
    let t = (seed += 0x6d2b79f5)
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

interface ProblemResult {
  problemIndex: number
  numWaypointPairs: number
  solveTimeMs: number
  score: number
  iterations: number
}

function runBenchmark() {
  const random = createSeededRandom(BENCHMARK_SEED)
  const results: ProblemResult[] = []
  const startTime = performance.now()

  console.log("=".repeat(60))
  console.log("ANGLED TRACE SOLVER BENCHMARK")
  console.log("=".repeat(60))
  console.log(`Running benchmark with ${NUM_PROBLEMS} problems...`)
  console.log(
    `Waypoint pairs range: ${MIN_WAYPOINT_PAIRS} to ${MAX_WAYPOINT_PAIRS}\n`,
  )

  for (let i = 0; i < NUM_PROBLEMS; i++) {
    const numWaypointPairs =
      MIN_WAYPOINT_PAIRS +
      Math.floor(random() * (MAX_WAYPOINT_PAIRS - MIN_WAYPOINT_PAIRS + 1))

    const problem = generateRandomProblem({
      randomSeed: i,
      numWaypointPairs,
      numObstacles: 2,
      minSpacing: 5,
    })

    const solver = new AngledTraceSolver(problem)
    const problemStart = performance.now()

    // Run solver until complete (with max iterations to prevent infinite loops)
    const MAX_ITERATIONS = 100
    let iterations = 0
    for (
      iterations = 0;
      iterations < MAX_ITERATIONS && !solver.solved;
      iterations++
    ) {
      solver.step()
    }

    const problemEnd = performance.now()
    const solveTimeMs = problemEnd - problemStart

    const score = scoreOutputCost({
      problem,
      outputTraces: solver.outputTraces,
    })

    results.push({
      problemIndex: i,
      numWaypointPairs,
      solveTimeMs,
      score,
      iterations,
    })

    // Progress indicator for each problem
    process.stdout.write(
      `\r  Progress: ${i + 1}/${NUM_PROBLEMS} problems completed`,
    )
  }
  console.log() // New line after progress

  const endTime = performance.now()
  const totalTimeMs = endTime - startTime

  // Calculate score statistics
  const scores = results.map((r) => r.score)
  const sortedScores = [...scores].sort((a, b) => a - b)
  const averageScore = scores.reduce((a, b) => a + b, 0) / scores.length
  const p95Index = Math.ceil(scores.length * 0.95) - 1
  const p95WorstScore = sortedScores[p95Index]
  const minScore = sortedScores[0]
  const maxScore = sortedScores[sortedScores.length - 1]
  const medianScore = sortedScores[Math.floor(scores.length / 2)]

  // Calculate timing statistics
  const times = results.map((r) => r.solveTimeMs)
  const sortedTimes = [...times].sort((a, b) => a - b)
  const avgTimeMs = times.reduce((a, b) => a + b, 0) / times.length
  const p95TimeMs = sortedTimes[Math.ceil(times.length * 0.95) - 1]
  const minTimeMs = sortedTimes[0]
  const maxTimeMs = sortedTimes[sortedTimes.length - 1]
  const medianTimeMs = sortedTimes[Math.floor(times.length / 2)]

  // Calculate iteration statistics
  const iterations = results.map((r) => r.iterations)
  const avgIterations =
    iterations.reduce((a, b) => a + b, 0) / iterations.length

  // Calculate stats by problem size
  const sizeGroups = new Map<number, ProblemResult[]>()
  for (const result of results) {
    const size = result.numWaypointPairs
    if (!sizeGroups.has(size)) {
      sizeGroups.set(size, [])
    }
    sizeGroups.get(size)!.push(result)
  }

  console.log("\n" + "=".repeat(60))
  console.log("OVERALL RESULTS")
  console.log("=".repeat(60))
  console.log(`Total problems:        ${NUM_PROBLEMS}`)
  console.log(`Total time:            ${(totalTimeMs / 1000).toFixed(2)}s`)
  console.log(`Avg iterations:        ${avgIterations.toFixed(1)}`)

  console.log("\n" + "-".repeat(60))
  console.log("TIMING STATISTICS (ms)")
  console.log("-".repeat(60))
  console.log(`Average:               ${avgTimeMs.toFixed(2)}`)
  console.log(`Median:                ${medianTimeMs.toFixed(2)}`)
  console.log(`P95 (slowest):         ${p95TimeMs.toFixed(2)}`)
  console.log(`Min (fastest):         ${minTimeMs.toFixed(2)}`)
  console.log(`Max (slowest):         ${maxTimeMs.toFixed(2)}`)

  console.log("\n" + "-".repeat(60))
  console.log("SCORE STATISTICS (lower is better)")
  console.log("-".repeat(60))
  console.log(`Average:               ${averageScore.toFixed(2)}`)
  console.log(`Median:                ${medianScore.toFixed(2)}`)
  console.log(`P95 (worst):           ${p95WorstScore.toFixed(2)}`)
  console.log(`Min (best):            ${minScore.toFixed(2)}`)
  console.log(`Max (worst):           ${maxScore.toFixed(2)}`)

  // Print breakdown by problem size
  console.log("\n" + "-".repeat(60))
  console.log("BREAKDOWN BY PROBLEM SIZE")
  console.log("-".repeat(60))
  console.log("Pairs  Count  Avg Time(ms)  Avg Score")
  console.log("-".repeat(60))

  const sortedSizes = [...sizeGroups.keys()].sort((a, b) => a - b)
  for (const size of sortedSizes) {
    const group = sizeGroups.get(size)!
    const groupAvgTime =
      group.reduce((a, r) => a + r.solveTimeMs, 0) / group.length
    const groupAvgScore = group.reduce((a, r) => a + r.score, 0) / group.length
    console.log(
      `${String(size).padStart(5)}  ${String(group.length).padStart(5)}  ${groupAvgTime.toFixed(2).padStart(12)}  ${groupAvgScore.toFixed(2).padStart(9)}`,
    )
  }

  console.log("=".repeat(60))

  // Print summary line suitable for comparison
  console.log(
    "\n[SUMMARY] AngledTraceSolver: " +
      `avg_time=${avgTimeMs.toFixed(2)}ms, ` +
      `avg_score=${averageScore.toFixed(2)}, ` +
      `p95_score=${p95WorstScore.toFixed(2)}`,
  )
}

runBenchmark()
