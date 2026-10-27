#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>
namespace hybrid_astar::ackermann {
namespace {
constexpr double infinity=std::numeric_limits<double>::infinity();
// Exact lower envelope of parabolas (q-i)^2 + f[i] with possible +inf seeds.
// Squared Euclidean distance to blocked cell CENTERS, expressed in grid cells.
void squaredDistance1D(const std::vector<double>& f,std::vector<double>& out) {
  const std::size_t n=f.size();
  std::vector<std::size_t> sites(n);
  std::vector<double> bounds(n+1);
  std::size_t count=0;
  for (std::size_t q=0;q<n;++q) {
    if (!std::isfinite(f[q])) continue;
    double crossing=-infinity;
    while(count>0) {
      const auto p=sites[count-1];
      // q > p, so denominator never zero.
      crossing=((f[q]+double(q)*double(q))-(f[p]+double(p)*double(p))) /
               (2.0*(double(q)-double(p)));
      if (crossing>bounds[count-1]) break;
      --count;
    }
    if (count==0) crossing=-infinity;
    sites[count]=q;
    bounds[count]=crossing;
    ++count;
  }
  out.assign(n,infinity);
  if (!count) return;
  std::size_t active=0;
  for (std::size_t q=0;q<n;++q) {
    while (active+1<count && bounds[active+1]<double(q)) ++active;
    const double delta=double(q)-double(sites[active]);
    out[q]=delta*delta+f[sites[active]];
  }
}
}
ConservativeDiscPrefilter::ConservativeDiscPrefilter(const MapSnapshot& m, OccupancyPolicy p,
                                                     Footprint footprint,std::size_t count,double inflationMeters)
 :map_(m) {
  // Reuse full map, occupancy and footprint validation; no bypass if invalid.
  ExactCollisionChecker checker(m,p,footprint,inflationMeters);
  if (count==0||count>64) throw std::invalid_argument("disc count must be 1..64");
  const double width=footprint.frontX-footprint.rearX;
  const double halfStrip=width/(2.0*count);
  radiusMeters_=std::hypot(footprint.halfWidth,halfStrip)+inflationMeters;
  if (!std::isfinite(radiusMeters_) || radiusMeters_<=0) throw std::invalid_argument("invalid disc cover");
  for(std::size_t j=0;j<count;++j)
    centers_.push_back(footprint.rearX+(double(j)+.5)*width/double(count));
  const std::size_t w=m.width,h=m.height;
  std::vector<double> horizontal(w*h,infinity);
  std::vector<double> line, output;
  for(std::size_t y=0;y<h;++y) {
    line.resize(w);
    for(std::size_t x=0;x<w;++x) {
      const auto value=m.cells[y*w+x];
      const bool blocked=value==-1 ? p.unknown!=UnknownPolicy::Free : value>p.maxTraversableOccupancy;
      line[x]=blocked?0.0:infinity;
    }
    squaredDistance1D(line,output);
    std::copy(output.begin(),output.end(),horizontal.begin()+y*w);
  }
  centerDistanceSquaredCells_.resize(w*h);
  for(std::size_t x=0;x<w;++x) {
    line.resize(h);
    for(std::size_t y=0;y<h;++y) line[y]=horizontal[y*w+x];
    squaredDistance1D(line,output);
    for(std::size_t y=0;y<h;++y) centerDistanceSquaredCells_[y*w+x]=output[y];
  }
}
bool ConservativeDiscPrefilter::definitelyClear(const Pose2D& pose) const {
  if (!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw)) return false;
  const double c0=std::cos(map_.origin.yaw),s0=std::sin(map_.origin.yaw);
  const double dx=pose.x-map_.origin.x,dy=pose.y-map_.origin.y;
  const double localX=c0*dx+s0*dy,localY=-s0*dx+c0*dy;
  const double theta=pose.yaw-map_.origin.yaw;
  const double ct=std::cos(theta),st=std::sin(theta),r=map_.resolutionMeters;
  // Query-center to cell-center + blocked-center to blocked-square corners.
  // Conservative bound is sqrt(2)*r; equality/tangency must FALL BACK.
  const double needed=radiusMeters_+std::numbers::sqrt2_v<double>*r+numericalMarginMeters_;
  const double mapWidth=double(map_.width)*r,mapHeight=double(map_.height)*r;
  for(const double offset:centers_) {
    const double x=localX+ct*offset,y=localY+st*offset;
    if (!std::isfinite(x)||!std::isfinite(y)||
        x<=radiusMeters_+numericalMarginMeters_ || y<=radiusMeters_+numericalMarginMeters_ ||
        x>=mapWidth-radiusMeters_-numericalMarginMeters_ ||
        y>=mapHeight-radiusMeters_-numericalMarginMeters_) return false;
    const auto xi=static_cast<std::size_t>(std::floor(x/r));
    const auto yi=static_cast<std::size_t>(std::floor(y/r));
    const double d2=centerDistanceSquaredCells_[yi*map_.width+xi];
    if (d2*r*r<=needed*needed) return false;
  }
  return true;
}
} // namespace hybrid_astar::ackermann
