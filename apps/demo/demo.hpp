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
  // Ackermann-only opt-in accepted-replacement limit: 8 (default), 12 or 16.
  // Does not alter the 160 attempt / 200000 strict pose-check ceilings.
  std::optional<std::size_t> maxAcceptedReplacements;
  bool curvedRefinement = false;
  bool steeringSmoothness = false; // Ackermann-only optional refinement objective
  double steeringWeightMetersSquared = 1.0; // applied only when checkbox is enabled
  bool footprintInflationEnabled = false;
  double footprintInflationMeters = 0.05; // UI value retained while disabled
  bool performanceHeuristic = true; // grid by default; false = certified Euclidean
  bool rsDistanceHeuristic = false; // extra RS distance guidance, requires the RS connector
  bool improve = false;
  bool ompl = false;
};
struct Outcome {
  ackermann::PlanRequest request;
  ackermann::PlanResult original;
  std::optional<ackermann::RefinementResult> refinement;
  bool rsDistanceHeuristicEnabled = false; // included in JSON for reproducible experiments
  double appliedSteeringWeightMetersSquared = 0.0; // effective weight, zero when off
  ackermann::RefinementOptions refinementSettings; // reported for reproducibility
  double elapsedMilliseconds = 0;
};
// Throws invalid_argument for unsupported scenarios and invalid pose requests.
ackermann::PlanRequest makeRequest(std::string_view scenario);
Outcome run(const Options& options);
std::string toJson(const Outcome& result);
std::string capabilitiesJson(const ackermann::MapSnapshot* customMap=nullptr);
} // namespace hybrid_astar::demo
