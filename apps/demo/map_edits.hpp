#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>
namespace hybrid_astar::demo {
struct CellEdit {
  std::size_t index = 0;       // row-major map-local cell index
  std::int8_t occupancy = 0;   // only -1, 0 or 100
};
// Strict canonical encoding: "index:value,index:value" with ascending indices.
// No duplicate indices or unbounded edit count permitted.
std::vector<CellEdit> parseCellEdits(std::string_view encoded);
// An edited request receives its own immutable MapSnapshot, never mutating
// the original map shared with other users/requests. Generation is supplied by
// the editor and must increase for each change, including a full reset.
std::shared_ptr<const ackermann::MapSnapshot> applyCellEdits(
    const std::shared_ptr<const ackermann::MapSnapshot>& base,
    const std::vector<CellEdit>& edits, std::uint64_t generation);
} // namespace hybrid_astar::demo
