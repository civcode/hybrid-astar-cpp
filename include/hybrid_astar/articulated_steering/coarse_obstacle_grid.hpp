#pragma once

#include "hybrid_astar/articulated_steering/planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <utility>
#include <vector>

namespace hybrid_astar::articulated_steering {

// Request-local, fully settled reverse-Dijkstra potential on an aligned,
// conservatively downsampled occupancy map. For guidance ONLY: coarse blocked
// tiles may conceal traversable narrow passages, and coarse distances are not
// certified lower bounds on continuous articulated driving cost.
//
// Any fine occupied / nontraversable-unknown cell blocks its entire coarse
// tile. The original fine map is never modified and remains the only source
// for search and strict two-body collision validation.
class CoarseObstacleGrid {
public:
  CoarseObstacleGrid(const ackermann::MapSnapshot& map,
                     ackermann::OccupancyPolicy policy,
                     const Pose2D& goal,double goalToleranceMeters,
                     double requestedResolutionMeters,Diagnostics& diagnostics)
      : map_(map), diagnostics_(diagnostics) {
    const auto started=std::chrono::steady_clock::now();
    // Integer alignment preserves all fine cells and handles arbitrary map
    // origins, rotations, and noninteger requested/native resolution ratios.
    factor_=std::max<std::size_t>(1,static_cast<std::size_t>(
        std::ceil(requestedResolutionMeters/map.resolutionMeters-1e-10)));
    resolution_=factor_*map.resolutionMeters;
    width_=(map.width-1)/factor_+1;
    height_=(map.height-1)/factor_+1;
    diagnostics_.coarseGridResolutionMeters=resolution_;
    diagnostics_.coarseGridCells=width_*height_;
    // Avoid unbounded auxiliary allocation if a caller selects a tiny
    // coarse resolution on an unusually large occupancy map.
    if(width_*height_>2'000'000) {
      diagnostics_.coarseGridSkipped=true;
      finishTiming(started);
      return;
    }
    std::vector<std::uint8_t> blocked(width_*height_,0);
    for(std::size_t fy=0;fy<map.height;++fy)
      for(std::size_t fx=0;fx<map.width;++fx) {
        const int cell=map.cells[fy*map.width+fx];
        if(cell<0 ? policy.unknown!=ackermann::UnknownPolicy::Free
                  : cell>policy.maxTraversableOccupancy)
          blocked[(fy/factor_)*width_+(fx/factor_)]=1;
      }
    for(const auto b:blocked)diagnostics_.coarseGridBlockedCells+=b;
    dist_.assign(blocked.size(),std::numeric_limits<double>::infinity());
    using Entry=std::pair<double,std::size_t>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> heap;
    const auto target=toLocal(goal.x,goal.y);
    const double t=std::max(0.0,goalToleranceMeters);
    if(target.first>=-t&&target.second>=-t&&
       target.first<=map.width*map.resolutionMeters+t&&
       target.second<=map.height*map.resolutionMeters+t) {
      const auto clampIndex=[&](double coordinate,std::size_t dimension) {
        return static_cast<std::size_t>(std::clamp(
            std::floor(coordinate/resolution_),0.0,double(dimension-1)));
      };
      const auto minX=clampIndex(target.first-t,width_);
      const auto maxX=clampIndex(target.first+t,width_);
      const auto minY=clampIndex(target.second-t,height_);
      const auto maxY=clampIndex(target.second+t,height_);
      for(std::size_t y=minY;y<=maxY;++y)
        for(std::size_t x=minX;x<=maxX;++x) {
          const std::size_t id=y*width_+x;
          if(blocked[id])continue;
          // Last tile may be smaller than nominal coarse cell size.
          const double left=x*resolution_,bottom=y*resolution_;
          const double right=std::min((x+1)*resolution_,
                                       map.width*map.resolutionMeters);
          const double top=std::min((y+1)*resolution_,
                                     map.height*map.resolutionMeters);
          const double dx=target.first-std::clamp(target.first,left,right);
          const double dy=target.second-std::clamp(target.second,bottom,top);
          if(std::hypot(dx,dy)<=t) {
            dist_[id]=0.0;
            heap.push({0.0,id});
            ++diagnostics_.coarseGridSeedCells;
          }
        }
    }
    // Full field once per request, with no 500-settlement per-query cap.
    // Prevent diagonal corner cutting as the existing LazyGrid does.
    constexpr double sqrt2=std::numbers::sqrt2_v<double>;
    while(!heap.empty()) {
      const auto [distance,index]=heap.top();heap.pop();
      if(distance>dist_[index])continue;
      ++diagnostics_.coarseGridSettlements;
      const std::size_t x=index%width_,y=index/width_;
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
        if(!dx&&!dy)continue;
        if((dx<0&&x==0)||(dx>0&&x+1>=width_)||
           (dy<0&&y==0)||(dy>0&&y+1>=height_))continue;
        const std::size_t nx=static_cast<std::size_t>(
            static_cast<std::ptrdiff_t>(x)+dx);
        const std::size_t ny=static_cast<std::size_t>(
            static_cast<std::ptrdiff_t>(y)+dy);
        const auto neighbor=ny*width_+nx;
        if(blocked[neighbor])continue;
        if(dx&&dy) {
          const auto sideX=y*width_+nx;
          const auto sideY=ny*width_+x;
          if(blocked[sideX]||blocked[sideY])continue;
        }
        const double candidate=distance+resolution_*
            (dx&&dy?sqrt2:1.0);
        if(candidate<dist_[neighbor]) {
          dist_[neighbor]=candidate;
          heap.push({candidate,neighbor});
        }
      }
    }
    finishTiming(started);
  }

  double estimate(double worldX,double worldY,double euclidean) noexcept {
    ++diagnostics_.coarseGridQueries;
    const auto xy=toLocal(worldX,worldY);
    if(dist_.empty()||xy.first<0.0||xy.second<0.0||
       xy.first>=map_.width*map_.resolutionMeters||
       xy.second>=map_.height*map_.resolutionMeters) {
      ++diagnostics_.coarseGridFallbacks;
      return euclidean;
    }
    const std::size_t x=static_cast<std::size_t>(xy.first/resolution_);
    const std::size_t y=static_cast<std::size_t>(xy.second/resolution_);
    const double value=dist_[y*width_+x];
    if(!std::isfinite(value)) {
      // Conservatively blocked coarse cells, disconnected pockets, or
      // goals without a free coarse seed must NEVER imply NoPath.
      ++diagnostics_.coarseGridFallbacks;
      return euclidean;
    }
    return std::max(euclidean,value);
  }

  double resolutionMeters() const noexcept {return resolution_;}
  std::size_t cellCount() const noexcept {return width_*height_;}
private:
  using Clock=std::chrono::steady_clock;
  void finishTiming(Clock::time_point started) {
    diagnostics_.coarseGridBuildMilliseconds=
        std::chrono::duration<double,std::milli>(Clock::now()-started).count();
  }
  std::pair<double,double> toLocal(double x,double y) const noexcept {
    const double dx=x-map_.origin.x,dy=y-map_.origin.y;
    const double c=std::cos(map_.origin.yaw),s=std::sin(map_.origin.yaw);
    return {c*dx+s*dy,-s*dx+c*dy};
  }
  const ackermann::MapSnapshot& map_;
  Diagnostics& diagnostics_;
  std::size_t factor_=1,width_=0,height_=0;
  double resolution_=0.0;
  std::vector<double> dist_;
};

} // namespace hybrid_astar::articulated_steering
