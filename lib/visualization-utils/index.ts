export const hashString = (str: string) => {
  let hash = 0
  for (let i = 0; i < str.length; i++) {
    hash = str.charCodeAt(i) * 779 + ((hash << 5) - hash)
  }
  return hash
}

export const getColorForNetworkId = (networkId?: string | null) => {
  if (!networkId) return "rgba(0, 0, 0, 0.5)"
  return `hsl(${hashString(networkId) % 360}, 100%, 50%)`
}

export { visualizeCurvyTraceProblem } from "./visualizeCurvyTraceProblem"
