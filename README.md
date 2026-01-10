# @tscircuit/curvy-trace-solver

Convert trace waypoints into curvy traces maximizing trace-to-trace and trace-to-obstacle distance

<img width="1352" height="1420" alt="image" src="https://github.com/user-attachments/assets/197d546a-6d2b-4bdc-8708-27a76430ff63" />

## Installation

```bash
bun add @tscircuit/curvy-trace-solver
```

## Usage

### Basic Example

```typescript
import { CurvyTraceSolver } from "@tscircuit/curvy-trace-solver"
import type { CurvyTraceProblem } from "@tscircuit/curvy-trace-solver"

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

const solver = new CurvyTraceSolver(problem)
solver.solve()

// Access the solved traces
console.log(solver.outputTraces)
// Each trace contains: { waypointPair, points: Point[], networkId? }
```

### Problem Definition

A `CurvyTraceProblem` consists of:

| Property           | Type             | Description                                                                 |
| ------------------ | ---------------- | --------------------------------------------------------------------------- |
| `bounds`           | `Bounds`         | The rectangular area containing the traces (`minX`, `minY`, `maxX`, `maxY`) |
| `waypointPairs`    | `WaypointPair[]` | Array of start/end point pairs to connect with traces                       |
| `obstacles`        | `Obstacle[]`     | Rectangular obstacles that traces should avoid                              |
| `preferredSpacing` | `number`         | Minimum desired spacing between traces                                      |

### Waypoint Pairs

```typescript
interface WaypointPair {
  start: { x: number; y: number }
  end: { x: number; y: number }
  networkId?: string // Traces with the same networkId are allowed to intersect
}
```

### Obstacles

```typescript
interface Obstacle {
  minX: number
  minY: number
  maxX: number
  maxY: number
  center: { x: number; y: number }
  networkId?: string // Traces with matching networkId can pass through this obstacle
}
```

### Output

After calling `solver.solve()`, access `solver.outputTraces` which contains:

```typescript
interface OutputTrace {
  waypointPair: WaypointPair
  points: Point[] // Array of points forming the curved trace
  networkId?: string
}
```

### Visualization

The solver provides a `visualize()` method that returns a `GraphicsObject` compatible with `graphics-debug`:

```typescript
import { getSvgFromGraphicsObject } from "graphics-debug"

const solver = new CurvyTraceSolver(problem)
solver.solve()

const svg = getSvgFromGraphicsObject(solver.visualize())
```
