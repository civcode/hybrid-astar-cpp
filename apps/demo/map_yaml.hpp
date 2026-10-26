#pragma once
#include "hybrid_astar/ackermann/planner.hpp"
#include <istream>
#include <memory>
#include <string>
namespace hybrid_astar::demo {
struct RosMapMetadata {
  std::string image;
  double resolutionMeters = 0;
  Pose2D origin{};
  double occupiedThreshold = .65;
  double freeThreshold = .196;
  bool negate = false;
};
// Restricted ROS map-server YAML: scalar keys plus [x,y,yaw], PGM only.
RosMapMetadata parseRosMapYaml(std::istream& input);
std::shared_ptr<ackermann::MapSnapshot> loadRosMapYaml(const std::string& filename);
}
