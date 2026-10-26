#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include <optional>
#include <memory>
#include <string>
#include <string_view>

namespace hybrid_astar::demo {
struct Options {
  std::string scenario = "open";
  std::shared_ptr<const ackermann::MapSnapshot> customMap;
  std::optional<Pose2D> start, goal;
  bool refine = false;
  bool curvedRefinement = false;
  bool performanceHeuristic = false;
  bool improve = false;
  bool ompl = false;
};
struct Outcome {
  ackermann::PlanRequest request;
  ackermann::PlanResult original;
  std::optional<ackermann::RefinementResult> refinement;
  double elapsedMilliseconds = 0;
};
// Throws invalid_argument for unsupported scenarios and invalid pose requests.
ackermann::PlanRequest makeRequest(std::string_view scenario);
Outcome run(const Options& options);
std::string toJson(const Outcome& result);
std::string capabilitiesJson(bool customMapAvailable=false);
} // namespace hybrid_astar::demo
