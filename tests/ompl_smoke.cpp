#include "hybrid_astar/steering/reeds_shepp.hpp"
#include <cmath>

int main() {
  using hybrid_astar::Pose2D;
  using hybrid_astar::steering::reedsSheppDistance;
  const Pose2D origin{};
  if (std::abs(reedsSheppDistance(origin, origin, 4.5)) > 1.e-8) return 1;
  if (std::abs(reedsSheppDistance(origin, Pose2D{1,0,0}, 4.5)-1.) > 1.e-6) return 2;
  if (std::abs(reedsSheppDistance(origin, Pose2D{-1,0,0}, 4.5)-1.) > 1.e-6) return 3;
  return 0;
}
