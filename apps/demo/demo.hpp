#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/refinement.hpp"
#include "apps/demo/map_edits.hpp"
#include <optional>
#include <memory>
#include <string>
#include <string_view>

namespace hybrid_astar::demo {
struct Options {
  std::string scenario = "open";
  std::string model = "ackermann";
  std::shared_ptr<const ackermann::MapSnapshot> customMap;
  std::vector<CellEdit> edits;
  std::uint64_t editGeneration = 0;
  std::optional<Pose2D> start, goal;
  // Optional work-budget override in expanded nodes. Empty keeps the
  // scenario default. Clamped by the callers to a demo-safe range.
  std::optional<std::size_t> maxExpansions;
  bool refine = false;
  bool curvedRefinement = false;
  bool performanceHeuristic = true; // grid by default; false = certified Euclidean
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
std::string capabilitiesJson(const ackermann::MapSnapshot* customMap=nullptr);
} // namespace hybrid_astar::demo
