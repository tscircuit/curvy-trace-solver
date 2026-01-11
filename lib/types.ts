import type { Point, Bounds } from "@tscircuit/math-utils"

// Point: { x: number, y: number }
// Bounds: { minX: number, minY: number, maxX: number, maxY: number }

export interface WaypointPair {
  start: Point
  end: Point

  // Waypoints may be part of the same network- in these cases they are allowed
  // to intersect with each other
  networkId?: string
}

export interface Obstacle extends Bounds {
  center: Point

  networkId?: string

  // Computed prior to the solver running
  outerSegments?: [Point, Point][]
}

export interface CurvyTraceProblem {
  bounds: Bounds
  waypointPairs: WaypointPair[]
  obstacles: Obstacle[]
  preferredTraceToTraceSpacing: number
  preferredObstacleToTraceSpacing: number
}

export interface OutputTrace {
  waypointPair: WaypointPair
  points: Point[]
  networkId?: string
}
