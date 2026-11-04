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
     inflationMeters_(inflationMeters),useDistanceTransform_(useDistanceTransform),
     originCos_(std::cos(m.origin.yaw)),originSin_(std::sin(m.origin.yaw)) {
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
void ClearanceScorer::buildDistanceTransform() {
  const std::size_t width=map_.width,height=map_.height;
  const double inf=std::numeric_limits<double>::infinity();
  const std::size_t longest=std::max(width,height);
  // A completely free map has infinite obstacle distance everywhere; only
  // the finite map boundary can produce a nonzero proximity penalty.
  bool hasObstacles=false;
  for(const auto cell:map_.cells) {
    if(isBlocked(cell,occupancy_)) {hasObstacles=true;break;}
  }
  if(!hasObstacles)return;
  std::vector<double> temporary(width*height);
  std::vector<double> seed(longest,inf),transformed(longest,inf);
  std::vector<std::size_t> sites(longest);
  std::vector<double> intersections(longest+1);
  for(std::size_t x=0;x<width;++x) {
    for(std::size_t y=0;y<height;++y)
      seed[y]=isBlocked(map_.cells[y*width+x],occupancy_)?0.0:inf;
    edt1d(seed,height,transformed,sites,intersections);
    for(std::size_t y=0;y<height;++y)
      temporary[y*width+x]=transformed[y];
  }
  blockedCenterDistanceSquared_.resize(width*height);
  for(std::size_t y=0;y<height;++y) {
    for(std::size_t x=0;x<width;++x)seed[x]=temporary[y*width+x];
    edt1d(seed,width,transformed,sites,intersections);
    for(std::size_t x=0;x<width;++x)
      blockedCenterDistanceSquared_[y*width+x]=transformed[x];
  }
}
bool ClearanceScorer::edtProvesZeroCost(const Pose2D& pose) const {
  // Convert the pose reference point to the same map-local coordinates as
  // the exact polygon scorer. Never approximate exact collision decisions.
  const double cs=originCos_,sn=originSin_;
  const double dx=pose.x-map_.origin.x,dy=pose.y-map_.origin.y;
  const double x0=cs*dx+sn*dy,y0=-sn*dx+cs*dy;
  const double r=map_.resolutionMeters;
  const double boundX=static_cast<double>(map_.width)*r;
  const double boundY=static_cast<double>(map_.height)*r;
  if(!std::isfinite(x0)||!std::isfinite(y0)||
     x0<0||y0<0||x0>=boundX||y0>=boundY)return false;
  const auto ix=static_cast<std::size_t>(std::floor(x0/r));
  const auto iy=static_cast<std::size_t>(std::floor(y0/r));
  if(ix>=map_.width||iy>=map_.height)return false;
  // Any point of the rotated body is <= bodyRadius_ from the reference pose.
  // Every blocked square lies within r/sqrt(2) of its center. The nearest
  // blocked-cell-center EDT, minus these radii and the query offset, is a
  // conservative LOWER bound on the true inflated-footprint clearance.
  const double offset=std::hypot(x0-(static_cast<double>(ix)+0.5)*r,
                                  y0-(static_cast<double>(iy)+0.5)*r);
  const double centerDist=blockedCenterDistanceSquared_.empty()?
      std::numeric_limits<double>::infinity():
      std::sqrt(blockedCenterDistanceSquared_[iy*map_.width+ix])*r;
  const double obstacleLower=centerDist-offset-
      r*std::numbers::sqrt2_v<double>*0.5-bodyRadius_-inflationMeters_;
  const double boundaryLower=std::min({x0,y0,boundX-x0,boundY-y0})-
                             bodyRadius_-inflationMeters_;
  // The guard also handles floating-point rounding of EDT roots, frame
  // rotation, and the conservative disk-minus-square geometry bound.
  const double guard=64*std::numeric_limits<double>::epsilon()*
      (1.0+std::abs(pose.x)+std::abs(pose.y)+
       std::abs(map_.origin.x)+std::abs(map_.origin.y)+
       boundX+boundY+bodyRadius_+inflationMeters_+influence_);
  return std::min(obstacleLower,boundaryLower)-guard>=influence_;
}
double ClearanceScorer::poseClearanceMeters(const Pose2D& pose,
                                            PlanDiagnostics* diagnostics) const {
  if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Nonfinite clearance pose");
  if(diagnostics)++diagnostics->clearanceDistanceQueries;
  if(useDistanceTransform_ && edtProvesZeroCost(pose)) {
    if(diagnostics)++diagnostics->clearanceEdtZeroCostBypasses;
    return influence_;
  }
  if(diagnostics)++diagnostics->clearanceExactFallbacks;
  return exactPoseClearanceMeters(pose);
}
double ClearanceScorer::exactPoseClearanceMeters(const Pose2D& pose) const {
  if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
    throw std::invalid_argument("Nonfinite clearance pose");
  const double c0=originCos_,s0=originSin_;
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
