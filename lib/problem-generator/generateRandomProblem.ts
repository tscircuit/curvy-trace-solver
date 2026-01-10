export const generateRandomProblem = (opts: {
  numWaypointPairs: number
  numObstacles: number
  randomSeed: number
}) => {
  if (opts.numObstacles > 0) throw new Error("Obstacles are not supported yet")

  return {
    // TODO
  }
}
