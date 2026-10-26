#include "hybrid_astar/ackermann/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hybrid_astar::ackermann {
namespace {
constexpr double pi = std::numbers::pi_v<double>;
constexpr double tau = 2.0 * pi;
constexpr double epsilon = 1e-10; // meters; map boundary/contact conservative
bool finite(const Pose2D& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
double wrap(double radians) { return std::remainder(radians, tau); }
struct Vec2 { double x, y; };
double dot(Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }

// Separating axes are the two square axes and two rectangle edge normals.
// Touching projections are INTERSECTION, never a safe separation.
bool overlapsSquare(const std::array<Vec2,4>& poly, double x, double y, double r) {
  const std::array<Vec2,4> square{{{x,y},{x+r,y},{x+r,y+r},{x,y+r}}};
  const Vec2 edge{poly[1].x-poly[0].x, poly[1].y-poly[0].y};
  const std::array<Vec2,4> axes{{{1,0},{0,1},{-edge.y,edge.x},edge}};
  for (auto axis : axes) {
    double pmin=std::numeric_limits<double>::infinity(),pmax=-pmin;
    double smin=pmin,smax=-pmin;
    for (auto v : poly) { double q=dot(axis,v); pmin=std::min(pmin,q); pmax=std::max(pmax,q); }
    for (auto v : square) { double q=dot(axis,v); smin=std::min(smin,q); smax=std::max(smax,q); }
    const double margin = epsilon * std::hypot(axis.x,axis.y);
    if (pmax < smin-margin || smax < pmin-margin) return false;
  }
  return true;
}
void validateMap(const MapSnapshot& m, OccupancyPolicy p) {
  if (m.width==0 || m.height==0 ||
      m.width > std::numeric_limits<std::size_t>::max()/m.height ||
      m.cells.size()!=m.width*m.height || !finite(m.origin) ||
      !std::isfinite(m.resolutionMeters) || m.resolutionMeters<=0 ||
      !std::isfinite(m.width*m.resolutionMeters) ||
      !std::isfinite(m.height*m.resolutionMeters) ||
      p.maxTraversableOccupancy<0 || p.maxTraversableOccupancy>100)
    throw std::invalid_argument("Invalid map geometry or occupancy policy");
  for (auto cell : m.cells)
    if (cell < -1 || cell > 100) throw std::invalid_argument("Malformed occupancy cell");
}
bool blocked(std::int8_t cell, OccupancyPolicy p) {
  return cell == -1 ? p.unknown != UnknownPolicy::Free : cell > p.maxTraversableOccupancy;
}
void validateFootprint(Footprint f) {
  if (!std::isfinite(f.rearX) || !std::isfinite(f.frontX) ||
      !std::isfinite(f.halfWidth) || f.rearX>=f.frontX || f.halfWidth<=0)
    throw std::invalid_argument("Invalid rectangular footprint");
}
void validateOptions(const PlanOptions& o) {
  validate(o.search);
  if (!std::isfinite(o.minimumTurningRadiusMeters) || o.minimumTurningRadiusMeters<=0 ||
      !std::isfinite(o.goalXYToleranceMeters) || o.goalXYToleranceMeters<0 ||
      !std::isfinite(o.goalYawToleranceRadians) || o.goalYawToleranceRadians<0 || o.goalYawToleranceRadians>pi ||
      !std::isfinite(o.searchSampleSpacingMeters) || o.searchSampleSpacingMeters<=0 ||
      !std::isfinite(o.finalSampleSpacingMeters) || o.finalSampleSpacingMeters<=0 ||
      !std::isfinite(o.searchMaxBodyPointMotionMeters) || o.searchMaxBodyPointMotionMeters<=0 ||
      !std::isfinite(o.finalMaxBodyPointMotionMeters) || o.finalMaxBodyPointMotionMeters<=0 ||
      o.maxSamplesPerSection<2 || o.maxExpansions==0 || o.maxGeneratedNodes==0)
    throw std::invalid_argument("Invalid planning options");
}
struct KeyHash {
  std::size_t operator()(const SearchKey& k) const noexcept {
    std::size_t h=std::hash<std::int64_t>{}(k.xBin);
    const auto mix=[&](std::size_t v) { h^=v+0x9e3779b97f4a7c15ULL+(h<<6)+(h>>2); };
    mix(std::hash<std::int64_t>{}(k.yBin)); mix(k.yawBin); mix(static_cast<unsigned>(k.incomingGear));
    return h;
  }
};
struct Node {
  Pose2D pose;
  SearchKey key;
  Gear gear;
  double g, h;
  std::size_t parent;
  MotionSection via;
};
struct Label { std::size_t node; double bestG; bool expanded; };
struct Entry { double f,h; std::size_t node; };
struct EntryWorse {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f!=b.f) return a.f>b.f;
    if (a.h!=b.h) return a.h>b.h;
    return a.node>b.node;
  }
};
using Queue=std::priority_queue<Entry,std::vector<Entry>,EntryWorse>;
double goalErrorXY(const Pose2D& p,const Pose2D& g) { return std::hypot(p.x-g.x,p.y-g.y); }
double goalErrorYaw(const Pose2D& p,const Pose2D& g) { return std::abs(wrap(p.yaw-g.yaw)); }
double heuristic(const Pose2D& p,const Pose2D& goal,double tolerance) {
  return std::max(0.0,goalErrorXY(p,goal)-tolerance);
}
enum class MotionStatus { Clear, Colliding, Incomplete };
MotionStatus checkMotion(const MotionSection& section, const ExactCollisionChecker& checker,
                         double step, double bodyStep, std::size_t limit, std::size_t& checks) {
  if (!std::isfinite(section.lengthMeters) || section.lengthMeters<0) return MotionStatus::Incomplete;
  // Upper bound on translation+rotation of EVERY body point per interval.
  const double factor=1.0+checker.maxBodyRadius()*std::abs(section.curvature);
  const double divisions=std::max(section.lengthMeters/step,section.lengthMeters*factor/bodyStep);
  if (!std::isfinite(divisions) || divisions > static_cast<double>(limit-1)) return MotionStatus::Incomplete;
  const std::size_t n=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(divisions)));
  for (std::size_t i=0;i<=n;++i) {
    if (checker.check(section.evaluate(section.lengthMeters*static_cast<double>(i)/n))==PoseCollision::Colliding)
      return MotionStatus::Colliding;
    ++checks;
  }
  return MotionStatus::Clear;
}
} // namespace

ExactCollisionChecker::ExactCollisionChecker(const MapSnapshot& map, OccupancyPolicy policy, Footprint footprint)
  : map_(map), policy_(policy), footprint_(footprint) {
  validateMap(map_,policy_); validateFootprint(footprint_);
  if (policy_.unknown==UnknownPolicy::Reject &&
      std::find(map_.cells.begin(),map_.cells.end(),static_cast<std::int8_t>(-1))!=map_.cells.end())
    throw std::domain_error("Unknown map rejected");
}

double ExactCollisionChecker::maxBodyRadius() const noexcept {
  return std::hypot(std::max(std::abs(footprint_.rearX),std::abs(footprint_.frontX)),footprint_.halfWidth);
}

PoseCollision ExactCollisionChecker::check(const Pose2D& pose) const {
  if (!finite(pose)) throw std::invalid_argument("Nonfinite collision pose");
  const double yaw=pose.yaw-map_.origin.yaw;
  const double c=std::cos(yaw),s=std::sin(yaw);
  const double co=std::cos(map_.origin.yaw),so=std::sin(map_.origin.yaw);
  const double dx=pose.x-map_.origin.x,dy=pose.y-map_.origin.y;
  const double x0=co*dx+so*dy,y0=-so*dx+co*dy;
  const std::array<Vec2,4> body{{{footprint_.rearX,-footprint_.halfWidth},
                                 {footprint_.frontX,-footprint_.halfWidth},
                                 {footprint_.frontX,footprint_.halfWidth},
                                 {footprint_.rearX,footprint_.halfWidth}}};
  std::array<Vec2,4> poly{};
  double minx=std::numeric_limits<double>::infinity(),miny=minx;
  double maxx=-minx,maxy=-minx;
  const double boundX=map_.width*map_.resolutionMeters,boundY=map_.height*map_.resolutionMeters;
  for (std::size_t i=0;i<4;++i) {
    auto& v=poly[i]; v={x0+c*body[i].x-s*body[i].y,y0+s*body[i].x+c*body[i].y};
    if (!std::isfinite(v.x)||!std::isfinite(v.y)) throw std::invalid_argument("Overflowed body transform");
    // Contacts with finite map edges are blocked, even Unknown::Free.
    if (v.x<=epsilon||v.y<=epsilon||v.x>=boundX-epsilon||v.y>=boundY-epsilon)
      return PoseCollision::Colliding;
    minx=std::min(minx,v.x);maxx=std::max(maxx,v.x);
    miny=std::min(miny,v.y);maxy=std::max(maxy,v.y);
  }
  const auto firstX=static_cast<std::size_t>(std::max(0.0,std::floor((minx-epsilon)/map_.resolutionMeters)));
  const auto firstY=static_cast<std::size_t>(std::max(0.0,std::floor((miny-epsilon)/map_.resolutionMeters)));
  const auto lastX=static_cast<std::size_t>(std::min(static_cast<double>(map_.width-1),std::floor((maxx+epsilon)/map_.resolutionMeters)));
  const auto lastY=static_cast<std::size_t>(std::min(static_cast<double>(map_.height-1),std::floor((maxy+epsilon)/map_.resolutionMeters)));
  for (std::size_t y=firstY;y<=lastY;++y) {
    for (std::size_t x=firstX;x<=lastX;++x) {
      if (blocked(map_.cells[y*map_.width+x],policy_) &&
          overlapsSquare(poly,x*map_.resolutionMeters,y*map_.resolutionMeters,map_.resolutionMeters))
        return PoseCollision::Colliding;
    }
  }
  return PoseCollision::Clear;
}

MotionSection makeSection(const Pose2D& p, Gear g, double curvature, double length) {
  if (!finite(p)||g==Gear::None||!std::isfinite(curvature)||!std::isfinite(length)||length<0)
    throw std::invalid_argument("Invalid Ackermann segment");
  MotionSection section{p,{},g,curvature,length};
  section.end=section.evaluate(length);
  return section;
}

Pose2D MotionSection::evaluate(double progress) const {
  if (!finite(start) || gear==Gear::None || !std::isfinite(curvature) ||
      !std::isfinite(lengthMeters) || lengthMeters<0 || !std::isfinite(progress) ||
      progress<0 || progress>lengthMeters)
    throw std::invalid_argument("Invalid Ackermann section evaluation");
  const double d=(gear==Gear::Forward?1.0:-1.0)*progress;
  const double delta=curvature*d;
  // Half-angle sinc form stays stable near kappa=0 and avoids subtractive cancellation.
  const double half=0.5*delta;
  const double sinc=std::abs(half)<1e-5 ? 1.0-half*half/6.0+half*half*half*half/120.0
                                         : std::sin(half)/half;
  const double travel=d*sinc;
  const double heading=start.yaw+half;
  const Pose2D p{start.x+travel*std::cos(heading),start.y+travel*std::sin(heading),
                 wrap(start.yaw+delta)};
  if (!finite(p)) throw std::out_of_range("Ackermann propagation overflow");
  return p;
}

PlanResult plan(const PlanRequest& r) {
  PlanResult result;
  if (r.map) { result.mapRevision=r.map->revision; result.mapFrameId=r.map->frameId; result.mapSourceId=r.map->sourceId; }
  result.effectiveOccupancy=r.occupancy;
  if (!r.map || !finite(r.start) || !finite(r.goal)) return result;
  try { validateOptions(r.options); validateFootprint(r.footprint); }
  catch (const std::exception&) { result.status=PlanStatus::InvalidRequest; return result; }
  try { validateMap(*r.map,r.occupancy); }
  catch (const std::exception&) { result.status=PlanStatus::InvalidMap; return result; }
  if (r.occupancy.unknown==UnknownPolicy::Reject &&
      std::find(r.map->cells.begin(),r.map->cells.end(),static_cast<std::int8_t>(-1))!=r.map->cells.end()) {
    result.status=PlanStatus::RejectedUnknownMap; return result;
  }
  try {
    const ExactCollisionChecker checker(*r.map,r.occupancy,r.footprint);
    if (checker.check(r.start)==PoseCollision::Colliding) { result.status=PlanStatus::StartInCollision; return result; }
    ++result.diagnostics.searchPoseChecks;
    const auto& opt=r.options;
    std::vector<Node> nodes;
    nodes.reserve(std::min<std::size_t>(opt.maxGeneratedNodes,20000));
    std::unordered_map<SearchKey,Label,KeyHash> active;
    Queue open;
    const auto startKey=quantize(r.start,Gear::None,opt.search);
    const double initialH=heuristic(r.start,r.goal,opt.goalXYToleranceMeters);
    nodes.push_back({r.start,startKey,Gear::None,0.0,initialH,0,{}});
    active.emplace(startKey,Label{0,0.0,false});
    open.push({initialH,initialH,0});
    result.diagnostics.generated=1;
    bool strictFailure=false;
    while (!open.empty()) {
      const Entry item=open.top();open.pop();
      const Node current=nodes[item.node]; // copy: nodes may grow while expanding
      auto label=active.find(current.key);
      if (label==active.end()||label->second.node!=item.node||label->second.expanded) {
        ++result.diagnostics.staleHeapPops;continue;
      }
      if (result.diagnostics.expanded>=opt.maxExpansions) {result.status=PlanStatus::BudgetExceeded;return result;}
      label->second.expanded=true;
      ++result.diagnostics.expanded;
      if (goalErrorXY(current.pose,r.goal)<=opt.goalXYToleranceMeters &&
          goalErrorYaw(current.pose,r.goal)<=opt.goalYawToleranceRadians) {
        std::vector<MotionSection> reverse;
        std::size_t id=item.node;
        while (id!=0) {reverse.push_back(nodes[id].via);id=nodes[id].parent;}
        std::reverse(reverse.begin(),reverse.end());
        bool valid=checker.check(r.start)==PoseCollision::Clear;
        ++result.diagnostics.finalPoseChecks;
        for (const auto& sec : reverse) {
          if (!valid) break;
          const auto status=checkMotion(sec,checker,opt.finalSampleSpacingMeters,
                         opt.finalMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                         result.diagnostics.finalPoseChecks);
          if (status==MotionStatus::Incomplete) {result.status=PlanStatus::ValidationIncomplete;return result;}
          valid=status==MotionStatus::Clear;
        }
        if (valid) {
          result.status=PlanStatus::Success;
          result.sections=std::move(reverse);
          result.reached=current.pose;result.finalGear=current.gear;
          result.weightedCostMeters=current.g;
          result.goalXYErrorMeters=goalErrorXY(current.pose,r.goal);
          result.goalYawErrorRadians=goalErrorYaw(current.pose,r.goal);
          result.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
          result.modelGeometryValid=true;
          return result;
        }
        strictFailure=true; // Never report an invalid candidate as Success.
      }
      const double curvature=1.0/opt.minimumTurningRadiusMeters;
      for (const Gear gear : {Gear::Forward,Gear::Reverse}) {
        for (double k : {-curvature,0.0,curvature}) {
          const auto sec=makeSection(current.pose,gear,k,opt.search.primitiveLengthMeters);
          const auto key=quantize(sec.end,gear,opt.search);
          const double cost=edgeCost(sec.lengthMeters,gear,current.gear,opt.search);
          const double g=current.g+cost;
          if (!std::isfinite(g)) {result.status=PlanStatus::InternalError;return result;}
          auto found=active.find(key);
          if (found!=active.end()&&found->second.bestG<=g) {
            ++result.diagnostics.prunedByKey;continue;
          }
          const auto check=checkMotion(sec,checker,opt.searchSampleSpacingMeters,
                     opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                     result.diagnostics.searchPoseChecks);
          if (check==MotionStatus::Incomplete) {result.status=PlanStatus::ValidationIncomplete;return result;}
          if (check==MotionStatus::Colliding) continue;
          if (nodes.size()>=opt.maxGeneratedNodes) {result.status=PlanStatus::BudgetExceeded;return result;}
          if (found!=active.end() && found->second.expanded) ++result.diagnostics.reopened;
          const double h=heuristic(sec.end,r.goal,opt.goalXYToleranceMeters);
          const auto id=nodes.size();
          nodes.push_back({sec.end,key,gear,g,h,item.node,sec});
          active[key]={id,g,false};
          open.push({g+h,h,id});
          ++result.diagnostics.generated;
        }
      }
    }
    result.status=strictFailure?PlanStatus::StrictValidationFailed:PlanStatus::NoPath;
  } catch (const std::exception&) {
    result.status=PlanStatus::InternalError;
  }
  return result;
}
} // namespace hybrid_astar::ackermann
