#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

// Euclidean distance between a separated convex polygon and an axis-aligned
// occupied map-cell square. Both are CLOSED geometric sets in metric units.
// Call the intersection test first; distance on separated shapes is obtained
// from polygon vertices to the square and square vertices to polygon edges.
// The geometry is continuous: no dilation or resampling of the grid occurs.
namespace hybrid_astar::ackermann::geometry {
struct Point2 {double x,y;};
inline double square(double v) {return v*v;}
inline double pointSegmentDistanceSquared(Point2 p,Point2 a,Point2 b) {
  const double dx=b.x-a.x,dy=b.y-a.y,denom=square(dx)+square(dy);
  const double t=denom>0?
      std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/denom,0.0,1.0):0.0;
  return square(p.x-a.x-t*dx)+square(p.y-a.y-t*dy);
}
template<std::size_t N>
double separatedConvexPolygonSquareDistanceSquared(
    const std::array<Point2,N>& polygon,double x,double y,double size) {
  static_assert(N>=3);
  const std::array<Point2,4> cell{{{x,y},{x+size,y},
                                   {x+size,y+size},{x,y+size}}};
  double distanceSquared=std::numeric_limits<double>::infinity();
  for(const auto p:polygon) {
    const double px=std::clamp(p.x,x,x+size);
    const double py=std::clamp(p.y,y,y+size);
    distanceSquared=std::min(distanceSquared,
                             square(p.x-px)+square(p.y-py));
  }
  for(const auto p:cell)
    for(std::size_t edge=0;edge<N;++edge)
      distanceSquared=std::min(distanceSquared,
          pointSegmentDistanceSquared(p,polygon[edge],polygon[(edge+1)%N]));
  return distanceSquared;
}
} // namespace hybrid_astar::ackermann::geometry
