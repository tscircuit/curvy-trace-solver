/**
 * Generate a random point on the boundary of the rectangle
 */
export function randomBoundaryPoint(
  rng: () => number,
  xmin: number,
  xmax: number,
  ymin: number,
  ymax: number,
): { x: number; y: number } {
  const W = xmax - xmin
  const H = ymax - ymin
  const perimeter = 2 * W + 2 * H
  const t = rng() * perimeter

  // Top edge: t in [0, W)
  if (t < W) {
    return { x: xmin + t, y: ymax }
  }
  // Right edge: t in [W, W + H)
  if (t < W + H) {
    return { x: xmax, y: ymax - (t - W) }
  }
  // Bottom edge: t in [W + H, 2W + H)
  if (t < 2 * W + H) {
    return { x: xmax - (t - W - H), y: ymin }
  }
  // Left edge: t in [2W + H, 2W + 2H)
  return { x: xmin, y: ymin + (t - 2 * W - H) }
}
