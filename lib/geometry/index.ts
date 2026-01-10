import type { Bounds } from "@tscircuit/math-utils"

export const getBoundsCenter = (bounds: Bounds) => {
  return {
    x: (bounds.minX + bounds.maxX) / 2,
    y: (bounds.minY + bounds.maxY) / 2,
  }
}
