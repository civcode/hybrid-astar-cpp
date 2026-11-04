#include "hybrid_astar/ackermann/clearance.hpp"
#include "hybrid_astar/ackermann/geometry_distance.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>
#include <limits>
#include <stdexcept>
namespace hybrid_astar::ackermann {
namespace {
using Point=geometry::Point2;
bool isBlocked(std::int8_t value,OccupancyPolicy policy) {
  return value==-1 ? policy.unknown!=UnknownPolicy::Free : value>policy.maxTraversableOccupancy;
}
}
// Exact squared Euclidean distance transform in 1D (lower envelope of
// squared-distance parabolas). INF entries are non-seeds. Reuse scratch space
// across rows/columns to avoid O(width*height) transient allocations.
void edt1d(const std::vector<double>& f,std::size_t n,
           std::vector<double>& out,std::vector<std::size_t>& sites,
           std::vector<double>& intersections) {
  constexpr double inf=std::numeric_limits<double>::infinity();
  std::size_t count=0;
  for(std::size_t q=0;q<n;++q) {
    if(!std::isfinite(f[q]))continue;
    double split=-inf;
    while(count>0) {
      const auto p=sites[count-1];
      const double dq=static_cast<double>(q),dp=static_cast<double>(p);
      split=((f[q]+dq*dq)-(f[p]+dp*dp))/(2.0*(dq-dp));
      if(split>intersections[count-1])break;
      --count;
    }
    if(count==0)split=-inf;
    sites[count]=q;
    intersections[count]=split;
    ++count;
    intersections[count]=inf;
  }
  if(count==0) {
    std::fill_n(out.begin(),n,inf);
    return;
  }
  std::size_t k=0;
  for(std::size_t q=0;q<n;++q) {
    while(k+1<count && intersections[k+1]<static_cast<double>(q))++k;
    const double d=static_cast<double>(q)-static_cast<double>(sites[k]);
    out[q]=d*d+f[sites[k]];
  }
}
ClearanceScorer::ClearanceScorer(const MapSnapshot& m,OccupancyPolicy p,Footprint f,
                                 double weight,double influence,bool useDistanceTransform,double inflationMeters)
    :map_(m),occupancy_(p),footprint_(f),lambda_(weight),influence_(influence),
     bodyRadius_(std::hypot(std::max(std::abs(f.rearX),std::abs(f.frontX)),f.halfWidth)),
     inflationMeters_(inflationMeters),useDistanceTransform_(useDistanceTransform) {
  ExactCollisionChecker check(m,p,f);
  if(!std::isfinite(weight)||weight<0||!std::isfinite(influence)||influence<=0||
     !std::isfinite(inflationMeters_)||inflationMeters_<0)
    throw std::invalid_argument("Invalid clearance cost configuration");
  if(useDistanceTransform_) {
    const auto start=std::chrono::steady_clock::now();
    buildDistanceTransform();
    buildMs_=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-start).count();
  }
}
double ClearanceScorer::exactPoseClearanceMeters(const Pose2D& pose) const {
  if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Nonfinite clearance pose");
  const double c0=std::cos(map_.origin.yaw),s0=std::sin(map_.origin.yaw);
  const double dx=pose.x-map_.origin.x,dy=pose.y-map_.origin.y;
  const double x0=c0*dx+s0*dy,y0=-s0*dx+c0*dy;
  const double theta=pose.yaw-map_.origin.yaw;
  const double c=std::cos(theta),s=std::sin(theta),r=map_.resolutionMeters;
  const std::array<Point,4> body{{{footprint_.rearX,-footprint_.halfWidth},
     {footprint_.frontX,-footprint_.halfWidth},{footprint_.frontX,footprint_.halfWidth},
     {footprint_.rearX,footprint_.halfWidth}}};
  std::array<Point,4> poly{};
  double minx=std::numeric_limits<double>::infinity(),miny=minx,maxx=-minx,maxy=-minx;
  for(std::size_t i=0;i<4;++i) {
    const auto p=body[i];poly[i]={x0+c*p.x-s*p.y,y0+s*p.x+c*p.y};
    minx=std::min(minx,poly[i].x);maxx=std::max(maxx,poly[i].x);
    miny=std::min(miny,poly[i].y);maxy=std::max(maxy,poly[i].y);
  }
  // The geometric inflation margin expands obstacles and map boundaries.
  // Exact and EDT-assisted variants use the SAME inflation-aware objective.
  const double physicalCutoff=influence_+inflationMeters_;
  double best=std::min({minx,miny,double(map_.width)*r-maxx,double(map_.height)*r-maxy,physicalCutoff});
  if(best<=0) return 0;
  const auto lowx=static_cast<std::size_t>(std::max(0.0,std::floor((minx-physicalCutoff)/r)));
  const auto lowy=static_cast<std::size_t>(std::max(0.0,std::floor((miny-physicalCutoff)/r)));
  const auto highx=static_cast<std::size_t>(std::min(double(map_.width-1),std::floor((maxx+physicalCutoff)/r)));
  const auto highy=static_cast<std::size_t>(std::min(double(map_.height-1),std::floor((maxy+physicalCutoff)/r)));
  if(lowx>highx||lowy>highy)
    return std::max(0.0,best-inflationMeters_);
  double bestSq=best*best;
  for(std::size_t y=lowy;y<=highy;++y) for(std::size_t x=lowx;x<=highx;++x) {
    if(!isBlocked(map_.cells[y*map_.width+x],occupancy_)) continue;
    const double lx=x*r,ly=y*r;
     bestSq=std::min(bestSq,
         geometry::separatedConvexPolygonSquareDistanceSquared(poly,lx,ly,r));
    if(bestSq<=0) return 0;
  }
  return std::max(0.0,std::sqrt(bestSq)-inflationMeters_);
}
double ClearanceScorer::sectionCost(const MotionSection& part,double step,double bodyStep,
                                     std::size_t limit,std::size_t& samples,
                                     PlanDiagnostics* diagnostics) const {
  const double len=part.lengthMeters;
  const double factor=1.0+bodyRadius_*std::abs(part.curvature);
  const double divisions=std::max(len/step,len*factor/bodyStep);
  if(!std::isfinite(divisions)||divisions>double(limit-1))
    throw std::invalid_argument("Clearance integration sample limit reached");
  const auto count=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(divisions)));
  double integral=0;
  for(std::size_t j=0;j<=count;++j) {
    const auto p=part.evaluate(j==count ? len : len*double(j)/double(count));
    const double distance=poseClearanceMeters(p,diagnostics);
    const double pen=std::max(0.0,1.0-distance/influence_);
    integral+=(j==0||j==count?.5:1.0)*pen*pen;
    ++samples;
  }
  const double cost=lambda_*len/double(count)*integral;
  if(!std::isfinite(cost)||cost<0) throw std::out_of_range("Clearance cost overflow");
  return cost;
}
} // namespace hybrid_astar::ackermann
