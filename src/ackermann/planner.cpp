#include "hybrid_astar/ackermann/planner.hpp"
#include "hybrid_astar/ackermann/primitives.hpp"
#include "hybrid_astar/planning/model_policy.hpp"
#include "hybrid_astar/ackermann/cover_prefilter.hpp"
#include "hybrid_astar/ackermann/clearance.hpp"
#include "hybrid_astar/ackermann/hotpath_profiler.hpp"

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
  steering::evaluateAnalyticCost(0.0,o.analytic);
  if (!std::isfinite(o.minimumTurningRadiusMeters) || o.minimumTurningRadiusMeters<=0 ||
      !std::isfinite(o.goalXYToleranceMeters) || o.goalXYToleranceMeters<0 ||
      !std::isfinite(o.goalYawToleranceRadians) || o.goalYawToleranceRadians<0 || o.goalYawToleranceRadians>pi ||
      !std::isfinite(o.searchSampleSpacingMeters) || o.searchSampleSpacingMeters<=0 ||
      !std::isfinite(o.finalSampleSpacingMeters) || o.finalSampleSpacingMeters<=0 ||
      !std::isfinite(o.searchMaxBodyPointMotionMeters) || o.searchMaxBodyPointMotionMeters<=0 ||
      !std::isfinite(o.finalMaxBodyPointMotionMeters) || o.finalMaxBodyPointMotionMeters<=0 ||
      o.maxSamplesPerSection<2 || o.maxFinalSamplesPerSection<2 ||
      o.maxTotalFinalPoseChecks==0 || o.maxExpansions==0 || o.maxGeneratedNodes==0 ||
      o.searchLabelReserve>o.maxGeneratedNodes ||
      o.discPrefilterCount>64 ||
      (o.gridQueueMode!=OpenQueueMode::BinaryLazy &&
       o.gridQueueMode!=OpenQueueMode::FourAryLazy &&
       o.gridQueueMode!=OpenQueueMode::IndexedBinary &&
       o.gridQueueMode!=OpenQueueMode::IndexedFourAry) ||
      (o.openQueueMode!=OpenQueueMode::BinaryLazy &&
       o.openQueueMode!=OpenQueueMode::FourAryLazy &&
       o.openQueueMode!=OpenQueueMode::IndexedBinary &&
       o.openQueueMode!=OpenQueueMode::IndexedFourAry) ||
      !std::isfinite(o.performanceHeuristicWeight) ||
      o.performanceHeuristicWeight<1 || o.performanceHeuristicWeight>3 ||
      !std::isfinite(o.clearanceWeight) || o.clearanceWeight<0 ||
      !std::isfinite(o.clearanceInfluenceRadiusMeters) || o.clearanceInfluenceRadiusMeters<=0 ||
      (o.solutionPolicy!=SolutionPolicy::FirstStrictlyValidated &&
       o.solutionPolicy!=SolutionPolicy::ImproveIncumbent))
    throw std::invalid_argument("Invalid planning options");
}
struct Label { std::size_t node; double bestG; bool expanded; };
double goalErrorXY(const Pose2D& p,const Pose2D& g) { return std::hypot(p.x-g.x,p.y-g.y); }
double goalErrorYaw(const Pose2D& p,const Pose2D& g) { return std::abs(wrap(p.yaw-g.yaw)); }
double heuristic(const Pose2D& p,const Pose2D& goal,double tolerance) {
  return std::max(0.0,goalErrorXY(p,goal)-tolerance);
}
// Deterministic request-local lazy reverse Dijkstra for PERFORMANCE ordering.
// Settled values are not certified continuous-space bounds or NoPath proofs.
class LazyGrid {
public:
  LazyGrid(const MapSnapshot& m,OccupancyPolicy p,const Pose2D& goal,
           double tolerance,std::size_t perQueryLimit,std::size_t totalLimit,
           bool preventCornerCutting,OpenQueueMode gridQueueMode,PlanDiagnostics& diag)
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
  Vec2 mapLocal(const Pose2D& p) const {
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
  PlanDiagnostics& diag_;
  std::vector<double> dist_;
  std::vector<bool> settled_;
  SearchOpenQueue heap_;
};
void summarizePath(PlanResult& r) {
  Gear previous=Gear::None;
  for (std::size_t i=0;i<r.sections.size();++i) {
    const auto& section=r.sections[i];
    r.geometricLengthMeters+=section.lengthMeters;
    if (section.gear==Gear::Reverse) r.reverseLengthMeters+=section.lengthMeters;
    if(section.kind==MotionKind::RotateInPlace)continue;
    if (previous!=Gear::None && previous!=section.gear) r.gearChangeSectionIndices.push_back(i);
    previous=section.gear;
  }
}
enum class MotionStatus { Clear, Colliding, Incomplete, Cancelled };
MotionStatus checkMotion(const MotionSection& section, const ExactCollisionChecker& checker,
                         double step, double bodyStep, std::size_t limit, std::size_t& checks,
                         const std::atomic_bool* cancel,
                         const ConservativeDiscPrefilter* prefilter=nullptr,
                         PlanDiagnostics* stats=nullptr,
                         std::size_t maxTotalChecks=std::numeric_limits<std::size_t>::max(),
                         bool finalPhase=false,
                         const motion::LocalPrimitive* templateMotion=nullptr) {
  std::uint64_t ignoredTime=0;
  HotPathTimer motionTimer(stats?(finalPhase?stats->finalMotionNanoseconds:
                                      stats->searchMotionNanoseconds):ignoredTime);
  if(stats) {
    if(finalPhase)++stats->finalMotionCalls;
    else ++stats->searchMotionCalls;
  }
  if (!std::isfinite(section.lengthMeters) || section.lengthMeters<0) return MotionStatus::Incomplete;
  // For a spin, reference travel is ZERO but every off-center body point
  // moves as much as radius*|yaw change|. Never use length-only sampling.
  const bool spin=section.kind==MotionKind::RotateInPlace;
  const double divisions=spin
      ? checker.maxBodyRadius()*std::abs(section.turnRadians)/bodyStep
      : std::max(section.lengthMeters/step,
                 section.lengthMeters*(1.0+checker.maxBodyRadius()*
                                     std::abs(section.curvature))/bodyStep);
  if (!std::isfinite(divisions) || divisions > static_cast<double>(limit-1)) return MotionStatus::Incomplete;
  const std::size_t n=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(divisions)));
  const bool useTemplate=templateMotion && templateMotion->samples.size()==n+1;
  // A single SE(2) anchor rotation suffices for every cached local sample.
  const double anchorCos=useTemplate?std::cos(section.start.yaw):0.0;
  const double anchorSin=useTemplate?std::sin(section.start.yaw):0.0;
  for (std::size_t i=0;i<=n;++i) {
    if (cancel && cancel->load(std::memory_order_relaxed)) return MotionStatus::Cancelled;
    if (checks>=maxTotalChecks) return MotionStatus::Incomplete;
    const auto pose=[&]() -> Pose2D {
      if(!useTemplate)
        return section.evaluate(i==n ? section.progressExtent() :
                                section.progressExtent()*static_cast<double>(i)/n);
      const auto& relative=templateMotion->samples[i].localPose;
      return {section.start.x+anchorCos*relative.x-anchorSin*relative.y,
              section.start.y+anchorSin*relative.x+anchorCos*relative.y,
              motion::wrapAngle(section.start.yaw+relative.yaw)};
    }();
    if(useTemplate && stats)++stats->primitiveTemplatePoseChecks;
    if (prefilter && prefilter->definitelyClear(pose)) {
      if (stats) ++stats->discDefinitelyClear;
    } else {
      if (prefilter && stats) ++stats->discExactFallback;
      PoseCollision status;
      if (stats) {
        ++stats->exactCollisionPoseCalls;
        HotPathTimer collisionTimer(stats->exactCollisionNanoseconds);
        status=checker.check(pose);
      } else status=checker.check(pose);
      if (status==PoseCollision::Colliding) return MotionStatus::Colliding;
    }
    ++checks;
  }
  return MotionStatus::Clear;
}
} // namespace

ExactCollisionChecker::ExactCollisionChecker(const MapSnapshot& map, OccupancyPolicy policy, Footprint footprint)
  : map_(map), policy_(policy), footprint_(footprint),
    originCos_(std::cos(map.origin.yaw)),originSin_(std::sin(map.origin.yaw)) {
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
  const double co=originCos_,so=originSin_;
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

MotionSection makeRotationSection(const Pose2D& start,double signedTurnRadians) {
  if(!finite(start)||!std::isfinite(signedTurnRadians)||signedTurnRadians==0)
    throw std::invalid_argument("Invalid stationary rotation section");
  MotionSection section;
  section.start=start;section.gear=Gear::None;
  section.kind=MotionKind::RotateInPlace;
  section.turnRadians=signedTurnRadians;
  section.end=section.evaluate(std::abs(signedTurnRadians));
  return section;
}

Pose2D MotionSection::evaluate(double progress) const {
  if(kind==MotionKind::RotateInPlace) {
    if(!finite(start)||!std::isfinite(turnRadians)||turnRadians==0||
       !std::isfinite(progress)||progress<0||progress>std::abs(turnRadians))
      throw std::invalid_argument("Invalid rotation section progress");
    return {start.x,start.y,wrap(start.yaw+std::copysign(progress,turnRadians))};
  }
  if (kind!=MotionKind::Translate||!finite(start) || gear==Gear::None ||
      !std::isfinite(curvature) || !std::isfinite(lengthMeters) ||
      lengthMeters<0 || !std::isfinite(progress) || progress<0 || progress>lengthMeters)
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

// This is ONE typed search kernel. The model supplies state indexing, cost,
// successor catalog and exact section materialization. Model dispatch occurs
// only once per request; the expansion loop contains no chassis switch.
template <planning::PlanarSearchModel Policy>
PlanResult planImpl(const PlanRequest& r,const Policy& model) {
  using Node=planning::SearchNode<typename Policy::State,typename Policy::Key,
                                  typename Policy::Mode,typename Policy::Section>;
  PlanResult result;
  if (r.map) { result.mapRevision=r.map->revision; result.mapFrameId=r.map->frameId; result.mapSourceId=r.map->sourceId; }
  result.effectiveOccupancy=r.occupancy;
  if (!r.map || !finite(r.start) || !finite(r.goal)) return result;
  try {
    validateOptions(r.options);
    validateFootprint(model.footprint());
    model.validate();
  } catch (const std::exception&) {
    result.status=PlanStatus::InvalidRequest;return result;
  }
  if (r.occupancy.maxTraversableOccupancy<0 || r.occupancy.maxTraversableOccupancy>100 ||
      (r.occupancy.unknown!=UnknownPolicy::Occupied &&
       r.occupancy.unknown!=UnknownPolicy::Free &&
       r.occupancy.unknown!=UnknownPolicy::Reject)) {
    result.status=PlanStatus::InvalidRequest;return result;
  }
  try { validateMap(*r.map,r.occupancy); }
  catch (const std::exception&) { result.status=PlanStatus::InvalidMap; return result; }
  if (r.occupancy.unknown==UnknownPolicy::Reject &&
      std::find(r.map->cells.begin(),r.map->cells.end(),static_cast<std::int8_t>(-1))!=r.map->cells.end()) {
    result.status=PlanStatus::RejectedUnknownMap; return result;
  }
  if (r.cancelled && r.cancelled->load(std::memory_order_relaxed)) {
    result.status=PlanStatus::Cancelled;return result;
  }
  try {
    const ExactCollisionChecker checker(*r.map,r.occupancy,model.footprint());
    std::unique_ptr<ConservativeDiscPrefilter> prefilter;
    if (r.options.discPrefilterCount)
      prefilter=std::make_unique<ConservativeDiscPrefilter>(
           *r.map,r.occupancy,model.footprint(),r.options.discPrefilterCount);
    std::unique_ptr<ClearanceScorer> clearance;
    if(r.options.clearanceWeight>0)
      clearance=std::make_unique<ClearanceScorer>(*r.map,r.occupancy,model.footprint(),
                 r.options.clearanceWeight,r.options.clearanceInfluenceRadiusMeters);
    const auto checkedPose=[&](const Pose2D& pose) {
      ++result.diagnostics.exactCollisionPoseCalls;
      HotPathTimer timer(result.diagnostics.exactCollisionNanoseconds);
      return checker.check(pose);
    };
    if (checkedPose(r.start)==PoseCollision::Colliding) { result.status=PlanStatus::StartInCollision; return result; }
    ++result.diagnostics.searchPoseChecks;
    const auto& opt=r.options;
    // Policy-specific catalog built once per request; reusable local samples
    // are rigidly transformed at every successor without integrating again.
    const motion::SampleLimits templateLimits{
      opt.searchSampleSpacingMeters,opt.searchMaxBodyPointMotionMeters,
      checker.maxBodyRadius(),std::numeric_limits<double>::max(),
      opt.maxSamplesPerSection};
    const auto primitiveCatalog=model.catalog(templateLimits);
    result.diagnostics.primitiveTemplatesBuilt=primitiveCatalog.size();
    std::vector<Node> nodes;
    nodes.reserve(std::min<std::size_t>(opt.maxGeneratedNodes,20000));
    std::unordered_map<typename Policy::Key,Label,typename Policy::Hash> active(
        0,model.keyHash(&result.diagnostics));
    if(opt.searchLabelReserve)active.reserve(opt.searchLabelReserve);
    result.diagnostics.labelPeakBuckets=active.bucket_count();
    SearchOpenQueue open(opt.openQueueMode);
    steering::AnalyticExpansionScheduler scheduler(opt.analytic);
    result.heuristicCertifiedAdmissible=opt.heuristicMode==HeuristicMode::CertifiedRegionEuclidean;
    std::unique_ptr<LazyGrid> grid;
    if (opt.heuristicMode==HeuristicMode::PerformanceReedsSheppAndObstacleGrid)
      grid=std::make_unique<LazyGrid>(*r.map,r.occupancy,r.goal,
                       opt.goalXYToleranceMeters,opt.gridMaxSettlementsPerQuery,
                       opt.gridMaxTotalSettlements,opt.gridPreventDiagonalCornerCutting,
                       opt.gridQueueMode,result.diagnostics);
    const auto estimate=[&](const Pose2D& p) {
      double h=heuristic(p,r.goal,opt.goalXYToleranceMeters);
      if (grid) {
        h=grid->estimate(p,h);
        if (r.analyticDistance) {
          try {
            const double d=r.analyticDistance(p,r.goal);
            if (std::isfinite(d)&&d>=0) h=std::max(h,d);
          } catch (const std::exception&) { /* keep finite fallbacks */ }
        }
      }
      return h;
    };
    // Inflation affects only the already noncertified performance ordering.
    // Coarse dominance and a raw-occupancy grid provide no suboptimality bound.
    const auto weightedEstimate=[&](const Pose2D& p) {
      const double raw=estimate(p);
      if (!grid || raw>std::numeric_limits<double>::max()/opt.performanceHeuristicWeight)
        return raw;
      return raw*opt.performanceHeuristicWeight;
    };
    const auto startKey=model.key(r.start,Gear::None);
    const double initialH=weightedEstimate(r.start);
    nodes.push_back({r.start,startKey,Gear::None,0.0,initialH,0.0,0,{}});
    const auto initialBucketCount=active.bucket_count();
    {
      HotPathTimer writeTimer(result.diagnostics.labelWriteNanoseconds);
      active.emplace(startKey,Label{0,0.0,false});
    }
    if(active.bucket_count()!=initialBucketCount)
      ++result.diagnostics.labelRehashes;
    ++result.diagnostics.labelInsertions;
    result.diagnostics.labelPeakSize=1;
    result.diagnostics.labelPeakBuckets=std::max(result.diagnostics.labelPeakBuckets,active.bucket_count());
    {
      HotPathTimer queueTimer(result.diagnostics.openQueueNanoseconds);
      open.push({initialH,initialH,0});
    }
    result.diagnostics.openQueuePushes=1;result.diagnostics.openQueuePeakSize=1;
    result.diagnostics.openQueuePeakStorageBytes=open.storageBytes();
    result.diagnostics.generated=1;
    bool strictFailure=false;
    PlanResult incumbent;
    bool hasIncumbent=false;
    std::size_t firstValidatedExpansion=0;
    const auto saveCandidate=[&](const std::vector<MotionSection>& sections,
                                 const Pose2D& reached,Gear gear,double cost,double clearCost) {
      ++result.diagnostics.validatedCandidates;
      if (hasIncumbent && cost>=incumbent.weightedCostMeters-1e-12) return;
      if (hasIncumbent) ++result.diagnostics.incumbentImprovements;
      incumbent=PlanResult{};
      incumbent.status=PlanStatus::Success;
      incumbent.sections=sections;
      incumbent.reached=reached;
      incumbent.finalGear=gear;
      incumbent.weightedCostMeters=cost;
      incumbent.clearanceCostMeters=clearCost;
      incumbent.goalXYErrorMeters=goalErrorXY(reached,r.goal);
      incumbent.goalYawErrorRadians=goalErrorYaw(reached,r.goal);
      incumbent.collisionEvidence=CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled;
      incumbent.modelGeometryValid=true;
      incumbent.heuristicCertifiedAdmissible=result.heuristicCertifiedAdmissible;
      incumbent.mapRevision=result.mapRevision;
      incumbent.mapFrameId=result.mapFrameId;
      incumbent.mapSourceId=result.mapSourceId;
      incumbent.effectiveOccupancy=result.effectiveOccupancy;
      incumbent.solutionQuality=result.diagnostics.incumbentImprovements
                             ?SolutionQuality::FeasibleAfterImprovement:SolutionQuality::FeasibleFirst;
      summarizePath(incumbent);
      if (!hasIncumbent) firstValidatedExpansion=result.diagnostics.expanded;
      hasIncumbent=true;
    };
    const auto finishIncumbent=[&](TerminationReason reason) {
      incumbent.terminationReason=reason;
      incumbent.diagnostics=result.diagnostics;
      return incumbent;
    };
    while (!open.empty()) {
      if (hasIncumbent &&
          result.diagnostics.expanded-firstValidatedExpansion >= opt.maxExtraExpansionsAfterFirst)
        return finishIncumbent(TerminationReason::ImprovementBudgetExhausted);
      if (r.cancelled && r.cancelled->load(std::memory_order_relaxed)) {
        if (hasIncumbent) return finishIncumbent(TerminationReason::CancelledAfterIncumbent);
        result.status=PlanStatus::Cancelled;return result;
      }
      const OpenEntry item=[&] {
        HotPathTimer queueTimer(result.diagnostics.openQueueNanoseconds);
        return open.pop();
      }();
      ++result.diagnostics.openQueuePops;
      // Copy only expansion fields, not the complete MotionSection. A
      // reference would dangle if the node vector reallocates for children.
      const auto& candidate=nodes[item.node];
      const struct {typename Policy::State pose; typename Policy::Key key;
                    typename Policy::Mode gear; double g,h,clearanceCost;} current{
        candidate.pose,candidate.key,candidate.gear,candidate.g,candidate.h,candidate.clearanceCost};
      const auto label=[&] {
        HotPathTimer lookupTimer(result.diagnostics.labelLookupNanoseconds);
        return active.find(current.key);
      }();
      ++result.diagnostics.labelFinds;
      if(label!=active.end())++result.diagnostics.labelFindHits;
      if (label==active.end()||label->second.node!=item.node||label->second.expanded) {
        ++result.diagnostics.staleHeapPops;continue;
      }
      if (result.diagnostics.expanded>=opt.maxExpansions) {
        if (hasIncumbent) return finishIncumbent(TerminationReason::WorkBudgetExceeded);
        result.status=PlanStatus::BudgetExceeded;return result;
      }
      label->second.expanded=true;
      ++result.diagnostics.expanded;
      if (goalErrorXY(current.pose,r.goal)<=opt.goalXYToleranceMeters &&
          goalErrorYaw(current.pose,r.goal)<=opt.goalYawToleranceRadians) {
        std::vector<MotionSection> reverse;
        std::size_t id=item.node;
        while (id!=0) {reverse.push_back(nodes[id].via);id=nodes[id].parent;}
        std::reverse(reverse.begin(),reverse.end());
        if(result.diagnostics.finalPoseChecks>=opt.maxTotalFinalPoseChecks) {
          if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
          result.status=PlanStatus::ValidationIncomplete;return result;
        }
        bool valid=checkedPose(r.start)==PoseCollision::Clear;
        ++result.diagnostics.finalPoseChecks;
        Pose2D continuousEnd=r.start;
        for (const auto& sec : reverse) {
          if (!valid) break;
          // Independent endpoint/continuity checks are essential for an
          // externally supplied model policy. Never trust a claimed section
          // end that does not agree with the section's exact evaluator.
          if(!finite(sec.start)||!finite(sec.end)||
             goalErrorXY(sec.start,continuousEnd)>1e-6||
             goalErrorYaw(sec.start,continuousEnd)>1e-6) {
            valid=false;break;
          }
          Pose2D actualEnd;
          try {actualEnd=sec.evaluate(sec.progressExtent());}
          catch(const std::exception&) {valid=false;break;}
          if(!finite(actualEnd) ||
             goalErrorXY(actualEnd,sec.end)>1e-6 ||
             goalErrorYaw(actualEnd,sec.end)>1e-6) {
            valid=false;break;
          }
          continuousEnd=actualEnd;
          const auto status=checkMotion(sec,checker,opt.finalSampleSpacingMeters,
                         opt.finalMaxBodyPointMotionMeters,opt.maxFinalSamplesPerSection,
                         result.diagnostics.finalPoseChecks,r.cancelled.get(),
                    nullptr,&result.diagnostics,opt.maxTotalFinalPoseChecks,true);
          if (status==MotionStatus::Cancelled) {if (hasIncumbent) return finishIncumbent(TerminationReason::CancelledAfterIncumbent);
            result.status=PlanStatus::Cancelled;return result;}
          if (status==MotionStatus::Incomplete) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
          valid=status==MotionStatus::Clear;
        }
        if(valid && (goalErrorXY(continuousEnd,current.pose)>1e-6 ||
                     goalErrorYaw(continuousEnd,current.pose)>1e-6 ||
                     goalErrorXY(continuousEnd,r.goal)>opt.goalXYToleranceMeters+1e-9 ||
                     goalErrorYaw(continuousEnd,r.goal)>opt.goalYawToleranceRadians+1e-9))
          valid=false;
        if (valid) {
          saveCandidate(reverse,continuousEnd,current.gear,current.g,current.clearanceCost);
          if (opt.solutionPolicy==SolutionPolicy::FirstStrictlyValidated)
            return finishIncumbent(TerminationReason::FirstValidated);
        }
        else strictFailure=true; // Never report an invalid candidate as Success.
      }
      // Optional analytic provider: one complete signed path. Search-bin cadence.
      if (r.analyticConnector && scheduler.shouldAttempt(current.h,opt.search.xyBinMeters)) {
        ++result.diagnostics.analyticAttempts;
        std::vector<MotionSection> proposed;
        try { proposed=r.analyticConnector(current.pose,r.goal); }
        catch (const std::exception&) { proposed.clear(); } // ordinary search survives
        if (!proposed.empty()) {
          std::vector<MotionSection> tail;
          tail.reserve(proposed.size());
          Pose2D expected=current.pose;
          Gear prior=current.gear;
          double weighted=current.g;
          double analyticClearance=current.clearanceCost;
          bool valid=true;
          for (const auto& part:proposed) {
            if (!finite(part.start)||!finite(part.end)||!std::isfinite(part.curvature)||
                !std::isfinite(part.lengthMeters)||part.lengthMeters<0||
                std::abs(part.curvature)>1.0/opt.minimumTurningRadiusMeters+1e-10||
                part.gear==Gear::None||
                goalErrorXY(part.start,expected)>1e-5||goalErrorYaw(part.start,expected)>1e-5) {
              valid=false;break;
            }
            const auto normalized=makeSection(expected,part.gear,part.curvature,part.lengthMeters);
            if (goalErrorXY(normalized.end,part.end)>1e-5 ||
                goalErrorYaw(normalized.end,part.end)>1e-5) {valid=false;break;}
            const double baseCost=edgeCost(normalized.lengthMeters,part.gear,prior,opt.search);
            if (!std::isfinite(weighted+baseCost)) {valid=false;break;}
            const auto check=checkMotion(normalized,checker,opt.searchSampleSpacingMeters,
                  opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                  result.diagnostics.searchPoseChecks,r.cancelled.get(),prefilter.get(),&result.diagnostics);
            if (check==MotionStatus::Cancelled) {if (hasIncumbent) return finishIncumbent(TerminationReason::CancelledAfterIncumbent);
            result.status=PlanStatus::Cancelled;return result;}
            if (check==MotionStatus::Incomplete) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
            if (check==MotionStatus::Colliding) {valid=false;break;}
            double extra=0;
            if(clearance) {
              try {extra=clearance->sectionCost(normalized,opt.searchSampleSpacingMeters,
                    opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                    result.diagnostics.clearanceSamples);}
              catch(const std::invalid_argument&) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
            }
            weighted+=baseCost+extra;
            analyticClearance+=extra;
            if(!std::isfinite(weighted)) {valid=false;break;}
            tail.push_back(normalized);
            expected=normalized.end;prior=part.gear;
          }
          if (valid && goalErrorXY(expected,r.goal)<=opt.goalXYToleranceMeters &&
              goalErrorYaw(expected,r.goal)<=opt.goalYawToleranceRadians &&
              steering::evaluateAnalyticCost(weighted-current.g,opt.analytic)==
                  steering::AnalyticCostGate::AcceptableForFurtherValidation) {
            std::vector<MotionSection> candidate;
            std::size_t id=item.node;
            while (id!=0) {candidate.push_back(nodes[id].via);id=nodes[id].parent;}
            std::reverse(candidate.begin(),candidate.end());
            candidate.insert(candidate.end(),tail.begin(),tail.end());
            if(result.diagnostics.finalPoseChecks>=opt.maxTotalFinalPoseChecks) {
              if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
              result.status=PlanStatus::ValidationIncomplete;return result;
            }
            bool finalValid=checkedPose(r.start)==PoseCollision::Clear;
            ++result.diagnostics.finalPoseChecks;
            for (const auto& part:candidate) {
              if (!finalValid) break;
              const auto check=checkMotion(part,checker,opt.finalSampleSpacingMeters,
                    opt.finalMaxBodyPointMotionMeters,opt.maxFinalSamplesPerSection,
                    result.diagnostics.finalPoseChecks,r.cancelled.get(),
                    nullptr,&result.diagnostics,opt.maxTotalFinalPoseChecks,true);
              if (check==MotionStatus::Cancelled) {if (hasIncumbent) return finishIncumbent(TerminationReason::CancelledAfterIncumbent);
            result.status=PlanStatus::Cancelled;return result;}
              if (check==MotionStatus::Incomplete) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
              finalValid=check==MotionStatus::Clear;
            }
            if (finalValid) {
              ++result.diagnostics.analyticAccepted;
              saveCandidate(candidate,expected,prior,weighted,analyticClearance);
              if (opt.solutionPolicy==SolutionPolicy::FirstStrictlyValidated)
                return finishIncumbent(TerminationReason::FirstValidated);
            }
            else strictFailure=true;
          }
        }
      }
      for (const auto& primitive : primitiveCatalog) {
          const auto* local=primitive.local.samples.empty()?nullptr:&primitive.local;
          const auto sec=model.materialize(current.pose,primitive);
          ++result.diagnostics.primitiveTemplateInstantiations;
          const auto nextGear=model.nextMode(current.gear,primitive);
          const auto key=model.key(sec.end,nextGear);
          const double cost=model.edgeCost(current.gear,primitive,sec);
          // Custom policies must never create zero/negative-cost cycles or
          // nonfinite priorities; a stationary turn has strictly positive cost.
          if(!std::isfinite(cost)||cost<=0) {
            result.status=PlanStatus::InvalidRequest;return result;
          }
          const double lowerG=current.g+cost;
          if (!std::isfinite(lowerG)) {result.status=PlanStatus::InternalError;return result;}
          const auto found=[&] {
            HotPathTimer lookupTimer(result.diagnostics.labelLookupNanoseconds);
            return active.find(key);
          }();
          ++result.diagnostics.labelFinds;
          if(found!=active.end())++result.diagnostics.labelFindHits;
          if (found!=active.end()&&found->second.bestG<=lowerG) {
            ++result.diagnostics.prunedByKey;continue;
          }
          const auto check=checkMotion(sec,checker,opt.searchSampleSpacingMeters,
                     opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                     result.diagnostics.searchPoseChecks,r.cancelled.get(),prefilter.get(),&result.diagnostics,
                     std::numeric_limits<std::size_t>::max(),false,local);
          if (check==MotionStatus::Cancelled) {if (hasIncumbent) return finishIncumbent(TerminationReason::CancelledAfterIncumbent);
            result.status=PlanStatus::Cancelled;return result;}
          if (check==MotionStatus::Incomplete) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
          if (check==MotionStatus::Colliding) continue;
          double extra=0;
          if (clearance) {
            try {extra=clearance->sectionCost(sec,opt.searchSampleSpacingMeters,
                      opt.searchMaxBodyPointMotionMeters,opt.maxSamplesPerSection,
                      result.diagnostics.clearanceSamples);}
            catch(const std::invalid_argument&) {if(hasIncumbent) return finishIncumbent(TerminationReason::ValidationBudgetExhausted);
            result.status=PlanStatus::ValidationIncomplete;return result;}
          }
          const double g=lowerG+extra;
          if (!std::isfinite(g)) {result.status=PlanStatus::InternalError;return result;}
          if (found!=active.end()&&found->second.bestG<=g) {
            ++result.diagnostics.prunedByKey;continue;
          }
          if (hasIncumbent && g>=incumbent.weightedCostMeters) {
            ++result.diagnostics.prunedByKey;continue; // nonnegative remaining cost
          }
          if (nodes.size()>=opt.maxGeneratedNodes) {
            if (hasIncumbent) return finishIncumbent(TerminationReason::WorkBudgetExceeded);
            result.status=PlanStatus::BudgetExceeded;return result;
          }
          if (found!=active.end() && found->second.expanded) ++result.diagnostics.reopened;
          const double h=weightedEstimate(sec.end);
          const auto id=nodes.size();
          nodes.push_back({sec.end,key,nextGear,g,h,current.clearanceCost+extra,item.node,sec});
          const auto oldNode=found!=active.end()?
              std::optional<std::size_t>(found->second.node):std::nullopt;
          const auto bucketsBefore=active.bucket_count();
          {
            HotPathTimer writeTimer(result.diagnostics.labelWriteNanoseconds);
            if(found!=active.end()) {
              found->second={id,g,false};
              ++result.diagnostics.labelUpdates;
            } else {
              active.emplace(key,Label{id,g,false});
              ++result.diagnostics.labelInsertions;
            }
          }
          if(active.bucket_count()!=bucketsBefore)++result.diagnostics.labelRehashes;
          result.diagnostics.labelPeakBuckets=
              std::max(result.diagnostics.labelPeakBuckets,active.bucket_count());
          result.diagnostics.labelPeakSize=
              std::max(result.diagnostics.labelPeakSize,active.size());
          const bool replaced=[&] {
            HotPathTimer queueTimer(result.diagnostics.openQueueNanoseconds);
            return open.push({g+h,h,id},oldNode);
          }();
          ++result.diagnostics.openQueuePushes;
          if(replaced)++result.diagnostics.openQueueReplacements;
          result.diagnostics.openQueuePeakSize=
              std::max(result.diagnostics.openQueuePeakSize,open.size());
          result.diagnostics.openQueuePeakStorageBytes=
              std::max(result.diagnostics.openQueuePeakStorageBytes,open.storageBytes());
          ++result.diagnostics.generated;
      }
    }
    if (hasIncumbent) return finishIncumbent(TerminationReason::SearchExhausted);
    result.status=strictFailure?PlanStatus::StrictValidationFailed:PlanStatus::NoPath;
  } catch (const std::exception&) {
    result.status=PlanStatus::InternalError;
  }
  return result;
}

PlanResult planWithPolicy(const PlanRequest& request,
                          const planning::PlanarPolicy& policy) {
  // An arbitrary model cannot silently inherit Ackermann-specific analytic
  // connectors or distance-only clearance integration. Future adapters may
  // expose independently validated model-specific connector capabilities.
  if(request.analyticConnector||request.analyticDistance||
     request.options.clearanceWeight>0) {
    PlanResult invalid;
    if(request.map) {
      invalid.mapRevision=request.map->revision;
      invalid.mapFrameId=request.map->frameId;
      invalid.mapSourceId=request.map->sourceId;
    }
    invalid.effectiveOccupancy=request.occupancy;
    invalid.status=PlanStatus::InvalidRequest;
    return invalid;
  }
  return planImpl(request,policy);
}

PlanResult plan(const PlanRequest& request) {
  switch(request.options.model) {
  case PlanOptions::Model::Ackermann:
    return planImpl(request,planning::AckermannPolicy{request});
  case PlanOptions::Model::DifferentialDrive:
    return planImpl(request,planning::DifferentialDrivePolicy{request});
  }
  PlanResult invalid;
  if(request.map) {
    invalid.mapRevision=request.map->revision;
    invalid.mapFrameId=request.map->frameId;
    invalid.mapSourceId=request.map->sourceId;
  }
  invalid.effectiveOccupancy=request.occupancy;
  invalid.status=PlanStatus::InvalidRequest;
  return invalid;
}
} // namespace hybrid_astar::ackermann
