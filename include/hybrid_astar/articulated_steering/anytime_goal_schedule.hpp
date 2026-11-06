#pragma once
#include <algorithm>
#include <cstddef>

namespace hybrid_astar::articulated_steering {

// Request-local anytime terminal-shot pacing. Slots spread attempts across
// the whole remaining expansion window. Exceptionally cheap lower-bound
// states can use a small fraction of the next slot early, never the entire
// shot budget immediately after the first validated solution.
inline bool anytimeGoalShotDue(std::size_t extraExpansions,
                              std::size_t shotsUsed,
                              std::size_t budgetExpansions,
                              std::size_t totalShots,
                              bool exceptionallyPromising=false) noexcept {
  if(totalShots==0||shotsUsed>=totalShots||budgetExpansions==0)
    return false;
  const std::size_t slot=(shotsUsed+1)*budgetExpansions/(totalShots+1);
  const std::size_t headroom=std::max<std::size_t>(
      1,budgetExpansions/(5*(totalShots+1)));
  return extraExpansions>=slot||
         (exceptionallyPromising&&extraExpansions+headroom>=slot);
}

} // namespace hybrid_astar::articulated_steering
