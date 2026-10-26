#include "hybrid_astar/ackermann/clearance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace hybrid_astar::ackermann {
namespace {
struct Point { double x,y; };
inline double sq(double x) {return x*x;}
// Squared distance from a point to a finite segment. For two closed convex
// polygons known not to overlap, min endpoint-to-edge distances is exact.
double toEdgeSquared(Point p,Point a,Point b) {
  const double dx=b.x-a.x,dy=b.y-a.y,denom=sq(dx)+sq(dy);
  const double t=denom>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/denom,0.0,1.0):0.0;
  return sq(p.x-a.x-t*dx)+sq(p.y-a.y-t*dy);
}
bool isBlocked(std::int8_t value,OccupancyPolicy policy) {
  return value==-1 ? policy.unknown!=UnknownPolicy::Free : value>policy.maxTraversableOccupancy;
}
}
ClearanceScorer::ClearanceScorer(const MapSnapshot& m,OccupancyPolicy p,Footprint f,
                                 double weight,double influence)
    :map_(m),occupancy_(p),footprint_(f),lambda_(weight),influence_(influence),
     bodyRadius_(std::hypot(std::max(std::abs(f.rearX),std::abs(f.frontX)),f.halfWidth)) {
  ExactCollisionChecker check(m,p,f);
  if(!std::isfinite(weight)||weight<0||!std::isfinite(influence)||influence<=0)
    throw std::invalid_argument("Invalid clearance cost configuration");
}
double ClearanceScorer::poseClearanceMeters(const Pose2D& pose) const {
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
  double best=std::min({minx,miny,double(map_.width)*r-maxx,double(map_.height)*r-maxy,influence_});
  if(best<=0) return 0;
  const auto lowx=static_cast<std::size_t>(std::max(0.0,std::floor((minx-influence_)/r)));
  const auto lowy=static_cast<std::size_t>(std::max(0.0,std::floor((miny-influence_)/r)));
  const auto highx=static_cast<std::size_t>(std::min(double(map_.width-1),std::floor((maxx+influence_)/r)));
  const auto highy=static_cast<std::size_t>(std::min(double(map_.height-1),std::floor((maxy+influence_)/r)));
  if(lowx>highx||lowy>highy) return best;
  double bestSq=best*best;
  for(std::size_t y=lowy;y<=highy;++y) for(std::size_t x=lowx;x<=highx;++x) {
    if(!isBlocked(map_.cells[y*map_.width+x],occupancy_)) continue;
    const double lx=x*r,ly=y*r;
    const std::array<Point,4> square{{{lx,ly},{lx+r,ly},{lx+r,ly+r},{lx,ly+r}}};
    for(const auto p:poly) {
      const double nearx=std::clamp(p.x,lx,lx+r),neary=std::clamp(p.y,ly,ly+r);
      bestSq=std::min(bestSq,sq(p.x-nearx)+sq(p.y-neary));
    }
    for(const auto p:square)
      for(std::size_t e=0;e<4;++e)
        bestSq=std::min(bestSq,toEdgeSquared(p,poly[e],poly[(e+1)%4]));
    if(bestSq<=0) return 0;
  }
  return std::sqrt(bestSq);
}
double ClearanceScorer::sectionCost(const MotionSection& part,double step,double bodyStep,
                                     std::size_t limit,std::size_t& samples) const {
  const double len=part.lengthMeters;
  const double factor=1.0+bodyRadius_*std::abs(part.curvature);
  const double divisions=std::max(len/step,len*factor/bodyStep);
  if(!std::isfinite(divisions)||divisions>double(limit-1))
    throw std::invalid_argument("Clearance integration sample limit reached");
  const auto count=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(divisions)));
  double integral=0;
  for(std::size_t j=0;j<=count;++j) {
    const auto p=part.evaluate(len*double(j)/double(count));
    const double distance=poseClearanceMeters(p);
    const double pen=std::max(0.0,1.0-distance/influence_);
    integral+=(j==0||j==count?.5:1.0)*pen*pen;
    ++samples;
  }
  const double cost=lambda_*len/double(count)*integral;
  if(!std::isfinite(cost)||cost<0) throw std::out_of_range("Clearance cost overflow");
  return cost;
}
} // namespace hybrid_astar::ackermann
