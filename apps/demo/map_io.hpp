#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <istream>
#include <memory>
#include <string>
namespace hybrid_astar::demo {
// PGM P2/P5 grayscale raster; black=blocked, white=free, 205=unknown
// for maxval 255. PGM top row maps to highest world Y (bottom-left origin).
// Rejects oversized/malformed data before allocating large map storage.
std::shared_ptr<ackermann::MapSnapshot> parsePgm(std::istream& in, double resolutionMeters);
std::shared_ptr<ackermann::MapSnapshot> loadPgm(const std::string& filename, double resolutionMeters);
}
