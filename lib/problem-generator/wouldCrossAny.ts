import { chordsCross } from "./countChordCrossings"

/**
 * Check if a new chord would cross any existing chords
 */
export function wouldCrossAny(
  newChord: [number, number],
  existingChords: [number, number][],
): boolean {
  for (const existing of existingChords) {
    if (chordsCross(newChord, existing)) {
      return true
    }
  }
  return false
}
