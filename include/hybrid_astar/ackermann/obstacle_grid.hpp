#pragma once

#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/hotpath_profiler.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace hybrid_astar::ackermann {
// Deterministic request-local lazy reverse Dijkstra for PERFORMANCE ordering.
// Settled values are not certified continuous-space bounds or NoPath proofs.
template <class Diagnostics>
class LazyGrid {
public:
  LazyGrid(const MapSnapshot& m,OccupancyPolicy p,const Pose2D& goal,
           double tolerance,std::size_t perQueryLimit,std::size_t totalLimit,
           bool preventCornerCutting,OpenQueueMode gridQueueMode,Diagnostics& diag)
      : m_(m),p_(p),perQueryLimit_(perQueryLimit),totalLimit_(totalLimit),
        preventCornerCutting_(preventCornerCutting),diag_(diag),heap_(gridQueueMode),
        dist_(m.cells.size(),std::numeric_limits<double>::infinity()),
        settled_(m.cells.size(),false) {
    const auto local=mapLocal(goal);
    const double r=m.resolutionMeters;
    // A cell intersects the goal disk iff its nearest square point is in the
    // disk. Only squares in the axis-aligned disk bounding box need testing.
    // Full-map seeding was O(width*height), even for a single-cell goal.
    const auto clipped=[&](double coordinate,std::size_t dim)->std::size_t {
      const double index=std::floor(coordinate/r);
      if(index<=0) return 0;
      if(index>=static_cast<double>(dim)) return dim-1;
      return static_cast<std::size_t>(index);
    };
    const double xmin=local.x-tolerance, xmax=local.x+tolerance;
    const double ymin=local.y-tolerance, ymax=local.y+tolerance;
    if(xmax<0||ymax<0||xmin>m.width*r||ymin>m.height*r)
      return; // no reachable goal-region seeds; later queries use fallback
    const auto firstX=clipped(xmin,m.width),lastX=clipped(xmax,m.width);
    const auto firstY=clipped(ymin,m.height),lastY=clipped(ymax,m.height);
    for(std::size_t y=firstY;y<=lastY;++y) {
      for(std::size_t x=firstX;x<=lastX;++x) {
        ++diag_.gridSeedCellsExamined;
        const std::size_t idx=y*m.width+x;
        if(!traversable(idx)) continue;
        const double left=x*r,bottom=y*r;
        const double dx=local.x-std::clamp(local.x,left,left+r);
        const double dy=local.y-std::clamp(local.y,bottom,bottom+r);
        if(std::hypot(dx,dy)<=tolerance) {
          dist_[idx]=0;
          {
            HotPathTimer timer(diag_.gridQueueNanoseconds);
            heap_.push({0.0,0.0,idx});
          }
          ++diag_.gridQueuePushes;
          diag_.gridQueuePeakSize=std::max(diag_.gridQueuePeakSize,heap_.size());
          diag_.gridQueuePeakStorageBytes=std::max(diag_.gridQueuePeakStorageBytes,heap_.storageBytes());
          ++diag_.gridSeedCellsAccepted;
        }
      }
    }
  }
  double estimate(const Pose2D& pose,double fallback) {
    const auto local=mapLocal(pose);
    if(local.x<0||local.y<0||local.x>=m_.width*m_.resolutionMeters||
       local.y>=m_.height*m_.resolutionMeters) {
      ++diag_.gridFallbacks;return fallback;
    }
    const auto x=static_cast<std::size_t>(std::floor(local.x/m_.resolutionMeters));
    const auto y=static_cast<std::size_t>(std::floor(local.y/m_.resolutionMeters));
    const auto target=y*m_.width+x;
    // A blocked center cell is not an exact footprint-collision oracle.
    // But it cannot have a finite value in this raw-grid relaxation; don't
    // burn hundreds of settlements trying to compute one.
    if(settled_[target]&&std::isfinite(dist_[target]))
      return std::max(fallback,dist_[target]);
    if(!traversable(target)||heap_.empty()) {
      if(!traversable(target))++diag_.gridBlockedQueryFallbacks;
      ++diag_.gridFallbacks;return fallback;
    }
    std::size_t work=0;
    while(work<perQueryLimit_&&diag_.gridSettlements<totalLimit_&&
          !heap_.empty()&&!settled_[target]) {
      const auto item=[&] {
        HotPathTimer timer(diag_.gridQueueNanoseconds);
        return heap_.pop();
      }();
      const double distance=item.f;const std::size_t index=item.node;
      ++diag_.gridQueuePops;
      if(settled_[index]||distance>dist_[index]) {++diag_.gridQueueStalePops;continue;}
      settled_[index]=true;++work;++diag_.gridSettlements;
      const std::size_t cx=index%m_.width,cy=index/m_.width;
      for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
        if(dx==0&&dy==0)continue;
        if((dx<0&&cx==0)||(dy<0&&cy==0)||
           (dx>0&&cx+1>=m_.width)||(dy>0&&cy+1>=m_.height))continue;
        if(preventCornerCutting_&&dx&&dy) {
          const std::size_t sideX=dx<0?index-1:index+1;
          const std::size_t sideY=dy<0?index-m_.width:index+m_.width;
          if(!traversable(sideX)||!traversable(sideY))continue;
        }
        std::size_t nx=cx,ny=cy;
        if(dx<0)--nx;else if(dx>0)++nx;
        if(dy<0)--ny;else if(dy>0)++ny;
        const std::size_t next=ny*m_.width+nx;
        if(!traversable(next)||settled_[next])continue;
        const double candidate=distance+m_.resolutionMeters*
                  ((dx&&dy)?std::numbers::sqrt2_v<double>:1.0);
        if(std::isfinite(candidate)&&candidate<dist_[next]) {
          dist_[next]=candidate;
          {
            HotPathTimer timer(diag_.gridQueueNanoseconds);
            heap_.push({candidate,candidate,next},next);
          }
          ++diag_.gridQueuePushes;
          diag_.gridQueuePeakSize=std::max(diag_.gridQueuePeakSize,heap_.size());
          diag_.gridQueuePeakStorageBytes=std::max(diag_.gridQueuePeakStorageBytes,heap_.storageBytes());
        }
      }
    }
    if(settled_[target]&&std::isfinite(dist_[target]))
      return std::max(fallback,dist_[target]);
    if(diag_.gridSettlements>=totalLimit_&&!heap_.empty())
      ++diag_.gridWorkBudgetFallbacks;
    ++diag_.gridFallbacks;
    return fallback;
  }
private:
  struct LocalPoint {double x,y;};
  LocalPoint mapLocal(const Pose2D& p) const {
    const double dx=p.x-m_.origin.x,dy=p.y-m_.origin.y;
    const double c=std::cos(m_.origin.yaw),s=std::sin(m_.origin.yaw);
    return {c*dx+s*dy,-s*dx+c*dy};
  }
  bool traversable(std::size_t index) const {
    return m_.cells[index]==-1?p_.unknown==UnknownPolicy::Free:
                               m_.cells[index]<=p_.maxTraversableOccupancy;
  }
  const MapSnapshot& m_;
  OccupancyPolicy p_;
  std::size_t perQueryLimit_,totalLimit_;
  bool preventCornerCutting_;
  Diagnostics& diag_;
  std::vector<double> dist_;
  std::vector<bool> settled_;
  SearchOpenQueue heap_;
};
} // namespace hybrid_astar::ackermann
