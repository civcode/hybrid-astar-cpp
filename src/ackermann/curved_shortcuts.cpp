#include "hybrid_astar/ackermann/curved_shortcuts.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace hybrid_astar::ackermann {
namespace {
constexpr double tau=2*std::numbers::pi_v<double>;
constexpr double angleTol=1e-6;
struct Point {double x, y;};
double positiveAngle(double x) {x=std::fmod(x,tau);return x<0?x+tau:x;}
Point leftNormal(double angle) {return {-std::sin(angle),std::cos(angle)};}
Point turnCenter(const Pose2D& pose,double radius,int turn) {
  const auto n=leftNormal(pose.yaw);
  return {pose.x+turn*radius*n.x,pose.y+turn*radius*n.y};
}
bool finite(const Pose2D& p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);
}
double totalLength(const std::vector<MotionSection>& sections) {
  double sum=0; for(const auto& s:sections) sum+=s.lengthMeters;return sum;
}
std::vector<MotionSection> connectForward(const Pose2D& from,const Pose2D& to,
                                         double r,int first,int last,double tol) {
  const auto a=turnCenter(from,r,first),b=turnCenter(to,r,last);
  const double dx=b.x-a.x,dy=b.y-a.y,D=std::hypot(dx,dy);
  const double offset=double(first-last)*r;
  if(D<1e-10 || std::abs(offset)>D) return {};
  const double direction=std::atan2(dy,dx)+std::asin(std::clamp(offset/D,-1.0,1.0));
  const double straight=std::sqrt(std::max(0.,D*D-offset*offset));
  const double t1=first>0?positiveAngle(direction-from.yaw):positiveAngle(from.yaw-direction);
  const double t2=last>0?positiveAngle(to.yaw-direction):positiveAngle(direction-to.yaw);
  std::vector<MotionSection> parts;
  Pose2D p=from;
  auto add=[&](double length,double curvature) {
    if(length<1e-9) return;
    parts.push_back(makeSection(p,Gear::Forward,curvature,length));
    p=parts.back().end;
  };
  add(t1*r,first/r);
  add(straight,0);
  add(t2*r,last/r);
  if(parts.empty() || std::hypot(p.x-to.x,p.y-to.y)>tol ||
     std::abs(std::remainder(p.yaw-to.yaw,tau))>angleTol) return {};
  return parts;
}
}
std::vector<std::vector<MotionSection>> generateCurvatureBoundedShortcuts(
    const Pose2D& start,const Pose2D& goal,double radius,double tolerance) {
  if(!finite(start)||!finite(goal)||!std::isfinite(radius)||radius<=0||
     !std::isfinite(tolerance)||tolerance<0)
    throw std::invalid_argument("Invalid curvature shortcut geometry");
  std::vector<std::vector<MotionSection>> candidates;
  // Reversal of vehicle travel is equivalent to forward motion of a pose
  // rotated by pi, provided physical curvature is negated on reconstruction.
  for(Gear gear:{Gear::Forward,Gear::Reverse}) {
    Pose2D a=start,b=goal;
    if(gear==Gear::Reverse) {a.yaw+=std::numbers::pi_v<double>;b.yaw+=std::numbers::pi_v<double>;}
    for(int turn1:{-1,1}) for(int turn2:{-1,1}) {
      const auto forward=connectForward(a,b,radius,turn1,turn2,tolerance);
      if(forward.empty()) continue;
      std::vector<MotionSection> actual;
      actual.reserve(forward.size());
      Pose2D p=start;
      for(const auto& section:forward) {
        const double curvature=gear==Gear::Forward?section.curvature:-section.curvature;
        auto arc=makeSection(p,gear,curvature,section.lengthMeters);
        p=arc.end;actual.push_back(arc);
      }
      if(std::hypot(p.x-goal.x,p.y-goal.y)<=tolerance&&
         std::abs(std::remainder(p.yaw-goal.yaw,tau))<=angleTol)
        candidates.push_back(std::move(actual));
    }
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){
    return totalLength(a)<totalLength(b);
  });
  return candidates;
}
} // namespace hybrid_astar::ackermann
