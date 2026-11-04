#include "hybrid_astar/ackermann/label_reserve_calibration.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hybrid_astar::ackermann {
namespace {
struct RandomStream {
  std::uint64_t state;
  explicit RandomStream(std::uint64_t seed):state(seed) {}
  std::uint64_t next() {
    state+=0x9e3779b97f4a7c15ULL;
    auto z=state;
    z=(z^(z>>30))*0xbf58476d1ce4e5b9ULL;
    z=(z^(z>>27))*0x94d049bb133111ebULL;
    return z^(z>>31);
  }
  double unit() {return static_cast<double>(next()>>11)*0x1.0p-53;}
};
struct Candidate {
  Pose2D pose;
  std::size_t stratum;
};
bool close(double a,double b) {
  return std::abs(a-b)<=1e-9*(1.0+std::abs(a)+std::abs(b));
}
bool samePose(const Pose2D& a,const Pose2D& b) {
  return close(a.x,b.x)&&close(a.y,b.y)&&close(a.yaw,b.yaw);
}
bool samePath(const PlanResult& a,const PlanResult& b) {
  if(a.status!=b.status||a.sections.size()!=b.sections.size()||
     !close(a.geometricLengthMeters,b.geometricLengthMeters)||
     !close(a.weightedCostMeters,b.weightedCostMeters)||
     !samePose(a.reached,b.reached))return false;
  for(std::size_t i=0;i<a.sections.size();++i) {
    const auto& x=a.sections[i];const auto& y=b.sections[i];
    if(x.kind!=y.kind||x.gear!=y.gear||
       !samePose(x.start,y.start)||!samePose(x.end,y.end)||
       !close(x.curvature,y.curvature)||
       !close(x.lengthMeters,y.lengthMeters)||
       !close(x.turnRadians,y.turnRadians)||
       !close(x.forwardMeters,y.forwardMeters)||
       !close(x.lateralMeters,y.lateralMeters))return false;
  }
  return true;
}
bool invalidStatus(PlanStatus s) {
  return s==PlanStatus::InvalidRequest||s==PlanStatus::InvalidMap||
         s==PlanStatus::RejectedUnknownMap||s==PlanStatus::InternalError||
         s==PlanStatus::Cancelled||s==PlanStatus::StartInCollision;
}
} // namespace

std::size_t labelReserveForTier(std::size_t tierIndex) {
  if(tierIndex==0)return 0;
  if(tierIndex>1+std::numeric_limits<std::size_t>::digits-17)
    throw std::out_of_range("Label reserve tier exceeds addressable size");
  std::size_t reserve=65536;
  for(std::size_t i=1;i<tierIndex;++i)reserve*=2;
  return reserve;
}
std::size_t labelReserveTierForPeak(std::size_t observedPeakLabels,
                                    double headroomFraction,
                                    std::size_t maxGeneratedNodes) {
  if(!std::isfinite(headroomFraction)||headroomFraction<0.0||
     headroomFraction>2.0||maxGeneratedNodes==0)
    throw std::invalid_argument("Invalid label reserve calibration headroom/budget");
  if(observedPeakLabels==0)
    throw std::invalid_argument("No observed state-label peak to calibrate");
  const long double requested=std::ceil(
      static_cast<long double>(observedPeakLabels)*
      (1.0L+static_cast<long double>(headroomFraction)));
  if(requested>static_cast<long double>(maxGeneratedNodes))
    throw std::out_of_range("Peak plus headroom exceeds maxGeneratedNodes");
  const auto needed=static_cast<std::size_t>(requested);
  for(std::size_t i=1;i<std::numeric_limits<std::size_t>::digits;++i) {
    const auto reserve=labelReserveForTier(i);
    if(reserve>maxGeneratedNodes)break;
    if(reserve>=needed)return i;
  }
  throw std::out_of_range(
      "No standard label reserve tier fits maxGeneratedNodes; increase budget "
      "or use a non-tier exact reserve");
}

LabelReserveCalibrationResult calibrateLabelReserve(
    const PlanRequest& prototype,const LabelReserveCalibrationOptions& settings) {
  if(!prototype.map)
    throw std::invalid_argument("Calibration requires a map snapshot");
  const auto& map=*prototype.map;
  if(map.width==0||map.height==0||map.cells.size()!=map.width*map.height||
     !std::isfinite(map.resolutionMeters)||map.resolutionMeters<=0 ||
     !std::isfinite(map.origin.x)||!std::isfinite(map.origin.y)||
     !std::isfinite(map.origin.yaw))
    throw std::invalid_argument("Calibration requires valid map geometry");
  if(settings.routeSamples==0||settings.routeSamples>10000||
     settings.poseAttemptsPerStratum==0||settings.poseAttemptsPerStratum>1000||
     settings.maxPairAttemptsPerRoute==0||settings.maxPairAttemptsPerRoute>1000||
     (settings.strataX==0)!=(settings.strataY==0)||
     !std::isfinite(settings.minStartGoalDistanceMeters)||
     settings.minStartGoalDistanceMeters<0)
    throw std::invalid_argument("Invalid calibration sample/coverage settings");
  // The function derives a recommended reserve from an unreserved search;
  // pre-existing caller options are copied but no reservation is inherited.
  PlanRequest request=prototype;
  request.options.searchLabelReserve=0;
  request.options.collectCollisionWork=false;
  if(request.cancelled)
    throw std::invalid_argument("Calibration expects no external cancellation token");

  LabelReserveCalibrationResult result;
  result.mapRevision=map.revision;
  result.mapWidth=map.width;result.mapHeight=map.height;
  result.requestedRoutes=settings.routeSamples;
  const double worldWidth=static_cast<double>(map.width)*map.resolutionMeters;
  const double worldHeight=static_cast<double>(map.height)*map.resolutionMeters;
  if(!std::isfinite(worldWidth)||!std::isfinite(worldHeight))
    throw std::invalid_argument("Map world dimensions overflow");
  if(settings.strataX) {
    result.strataX=settings.strataX;result.strataY=settings.strataY;
  } else {
    const double target=std::max(4.0,2.0*static_cast<double>(settings.routeSamples));
    const double x=std::sqrt(target*worldWidth/worldHeight);
    result.strataX=std::min(map.width,std::max<std::size_t>(
        2,static_cast<std::size_t>(std::ceil(x))));
    result.strataY=std::min(map.height,std::max<std::size_t>(
        2,static_cast<std::size_t>(std::ceil(target/result.strataX))));
  }
  if(result.strataX==0||result.strataY==0||
     result.strataX>map.width||result.strataY>map.height||
     result.strataX>4096/result.strataY)
    throw std::invalid_argument("Calibration strata exceed map grid or 4096-bin limit");
  const std::size_t totalStrata=result.strataX*result.strataY;
  const double minimumDistance=settings.minStartGoalDistanceMeters>0.0?
      settings.minStartGoalDistanceMeters:0.15*std::hypot(worldWidth,worldHeight);
  const double cs=std::cos(map.origin.yaw),sn=std::sin(map.origin.yaw);
  const ExactCollisionChecker checker(map,request.occupancy,request.footprint,
      request.options.footprintInflationMeters,
      request.options.useEmptyRegionPrefixSum,
      request.options.useInflatedFootprintBroadPhase,
      request.options.useFastEmptyAabb);
  RandomStream rng(settings.seed);
  std::vector<Candidate> candidates;
  candidates.reserve(totalStrata);
  // Generate one random feasible heading/pose in each geographically distinct
  // map-local stratum. Unlike unrestricted uniform sampling, obstacles cannot
  // cause all candidates to concentrate in a few large free regions.
  for(std::size_t y=0;y<result.strataY;++y) {
    for(std::size_t x=0;x<result.strataX;++x) {
      const double minX=worldWidth*static_cast<double>(x)/result.strataX;
      const double maxX=worldWidth*static_cast<double>(x+1)/result.strataX;
      const double minY=worldHeight*static_cast<double>(y)/result.strataY;
      const double maxY=worldHeight*static_cast<double>(y+1)/result.strataY;
      for(std::size_t attempt=0;attempt<settings.poseAttemptsPerStratum;++attempt) {
        const double lx=minX+(maxX-minX)*rng.unit();
        const double ly=minY+(maxY-minY)*rng.unit();
        const Pose2D p{map.origin.x+cs*lx-sn*ly,
                       map.origin.y+sn*lx+cs*ly,
                       -std::numbers::pi_v<double>+
                          2.0*std::numbers::pi_v<double>*rng.unit()};
        if(checker.check(p)==PoseCollision::Clear) {
          candidates.push_back({p,y*result.strataX+x});
          break;
        }
      }
    }
  }
  result.validStrata=candidates.size();
  if(candidates.size()<2)
    throw std::runtime_error("Fewer than two strata contain collision-free sampled poses");
  // Shuffle the valid strata. Cyclic start indices and a moving goal offset
  // distribute both endpoint roles across the map. Distance and unique-pair
  // filters prevent a few trivial local routes dominating the calibration.
  for(std::size_t i=candidates.size();i>1;--i)
    std::swap(candidates[i-1],candidates[static_cast<std::size_t>(rng.next()%i)]);
  std::set<std::pair<std::size_t,std::size_t>> seenPairs;
  std::vector<bool> startCovered(totalStrata),goalCovered(totalStrata);
  std::vector<PlanResult> baselinePlans;
  if(settings.verifyNoRehash)baselinePlans.reserve(settings.routeSamples);
  const std::size_t maxAttempts=settings.routeSamples*settings.maxPairAttemptsPerRoute;
  for(std::size_t attempt=0;
      attempt<maxAttempts && result.trials.size()<settings.routeSamples;++attempt) {
    const auto startIndex=attempt%candidates.size();
    const auto& start=candidates[startIndex];
    std::size_t goalIndex=candidates.size();
    for(std::size_t offset=0;offset<candidates.size();++offset) {
      const auto idx=(startIndex+candidates.size()/2+
                      (attempt/candidates.size())*3+offset)%candidates.size();
      if(idx==startIndex)continue;
      const auto& goal=candidates[idx];
      if(std::hypot(goal.pose.x-start.pose.x,goal.pose.y-start.pose.y)<
         minimumDistance)continue;
      if(seenPairs.count({start.stratum,goal.stratum}))continue;
      goalIndex=idx;break;
    }
    if(goalIndex==candidates.size())continue;
    const auto& goal=candidates[goalIndex];
    seenPairs.insert({start.stratum,goal.stratum});
    request.start=start.pose;
    request.goal=goal.pose;
    const auto raw=plan(request);
    if(invalidStatus(raw.status))
      throw std::runtime_error("Calibration search failed validation or was cancelled");
    if(raw.diagnostics.labelPeakSize==0)
      throw std::runtime_error("Calibration produced no live state labels");
    LabelReserveCalibrationTrial trial;
    trial.start=start.pose;trial.goal=goal.pose;
    trial.startStratum=start.stratum;trial.goalStratum=goal.stratum;
    trial.status=raw.status;
    trial.expanded=raw.diagnostics.expanded;
    trial.generated=raw.diagnostics.generated;
    trial.peakLabels=raw.diagnostics.labelPeakSize;
    trial.baselineRehashes=raw.diagnostics.labelRehashes;
    result.maxObservedPeakLabels=std::max(result.maxObservedPeakLabels,trial.peakLabels);
    if(raw.status==PlanStatus::Success)++result.successfulRoutes;
    if(raw.status==PlanStatus::BudgetExceeded||
       raw.status==PlanStatus::ValidationIncomplete)
      ++result.budgetLimitedRoutes;
    startCovered[trial.startStratum]=true;
    goalCovered[trial.goalStratum]=true;
    result.trials.push_back(trial);
    if(settings.verifyNoRehash)baselinePlans.push_back(raw);
  }
  result.evaluatedRoutes=result.trials.size();
  result.coveredStartStrata=static_cast<std::size_t>(
      std::count(startCovered.begin(),startCovered.end(),true));
  result.coveredGoalStrata=static_cast<std::size_t>(
      std::count(goalCovered.begin(),goalCovered.end(),true));
  result.sampleTargetMet=result.evaluatedRoutes==settings.routeSamples;
  if(!result.sampleTargetMet)
    throw std::runtime_error("Insufficient distinct valid start/goal pairs; "
                             "lower --samples/--min-distance, increase strata "
                             "or use a larger map");
  result.recommendedTierIndex=labelReserveTierForPeak(
      result.maxObservedPeakLabels,settings.headroomFraction,
      request.options.maxGeneratedNodes);
  result.recommendedReserveLabels=labelReserveForTier(result.recommendedTierIndex);
  if(settings.verifyNoRehash) {
    request.options.searchLabelReserve=result.recommendedReserveLabels;
    for(std::size_t i=0;i<result.trials.size();++i) {
      request.start=result.trials[i].start;
      request.goal=result.trials[i].goal;
      const auto verified=plan(request);
      result.trials[i].verifiedRehashes=verified.diagnostics.labelRehashes;
      const auto& baseline=baselinePlans[i];
      if(verified.diagnostics.labelRehashes!=0||
         verified.diagnostics.labelPeakSize!=baseline.diagnostics.labelPeakSize||
         verified.diagnostics.expanded!=baseline.diagnostics.expanded||
         verified.diagnostics.generated!=baseline.diagnostics.generated||
         verified.diagnostics.exactCollisionPoseCalls!=
             baseline.diagnostics.exactCollisionPoseCalls||
         verified.diagnostics.labelFinds!=baseline.diagnostics.labelFinds||
         verified.diagnostics.labelInsertions!=baseline.diagnostics.labelInsertions||
         verified.diagnostics.labelUpdates!=baseline.diagnostics.labelUpdates||
         !samePath(baseline,verified))
        throw std::runtime_error("Tier replay changed search behavior or caused rehash");
    }
    result.verificationPassed=true;
  }
  return result;
}
} // namespace hybrid_astar::ackermann
