// Explicit/manual full-resolution Nav2 clearance-cost A/B/C benchmark.
// Identical routes and footprints. Exact and EDT-assisted objectives MUST
// produce identical paths and search work; clearance-OFF may differ.
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include "hybrid_astar/ackermann/clearance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pipeline=hybrid_astar::bench::nav2_pipeline;
namespace fe=hybrid_astar::bench::footprint_envelope;
using namespace hybrid_astar::ackermann;
namespace {
enum class Variant { Off, Exact, Edt };
const char* label(Variant v) {
  switch(v) {
  case Variant::Off:return "off";
  case Variant::Exact:return "exact";
  case Variant::Edt:return "edt";
  }
  return "invalid";
}
double positiveReal(const std::string& text,bool allowZero=false) {
  std::size_t used=0;
  const double v=std::stod(text,&used);
  if(used!=text.size()||!std::isfinite(v)||(allowZero?v<0:v<=0))
    throw std::invalid_argument("Expected finite positive clearance parameter");
  return v;
}
struct Quality {
  double minClearance=-1.0,p10Clearance=-1.0;
  std::size_t poses=0;
};
Quality assessRawPath(const PlanRequest& req,const PlanResult& raw) {
  if(!pipeline::strictSuccess(raw))return {};
  // Independent UNTIMED exact clearance metrics, saturated at 1.0 m.
  // Use identical sampling for all variants and report inflated clearance.
  const ClearanceScorer exact(*req.map,req.occupancy,req.footprint,
      0.,1.0,false,req.options.footprintInflationMeters);
  std::vector<double> values;
  for(const auto& section:raw.sections) {
    const double extent=section.progressExtent();
    const auto n=std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(extent/.2)));
    if(n>100000)throw std::runtime_error("Path quality sample cap exceeded");
    for(std::size_t i=0;i<=n;++i)
      values.push_back(exact.poseClearanceMeters(
          section.evaluate(i==n?extent:extent*static_cast<double>(i)/n)));
  }
  if(values.empty())return {};
  std::sort(values.begin(),values.end());
  const std::size_t p10=std::min(values.size()-1,
      static_cast<std::size_t>(std::ceil(values.size()*.10))-1);
  return {values.front(),values[p10],values.size()};
}
void checkEquivalent(const pipeline::Measurement& exact,
                     const pipeline::Measurement& edt) {
  if(!pipeline::equivalentMeasurements(exact,edt))
    throw std::runtime_error("EDT changed exact-objective search/refinement path");
  const auto& a=exact.raw.diagnostics;
  const auto& b=edt.raw.diagnostics;
  if(a.clearanceSamples!=b.clearanceSamples ||
     a.clearanceDistanceQueries!=b.clearanceDistanceQueries ||
     a.labelFinds!=b.labelFinds ||
     a.labelInsertions!=b.labelInsertions ||
     a.prunedByKey!=b.prunedByKey ||
     a.staleHeapPops!=b.staleHeapPops ||
     a.openQueuePops!=b.openQueuePops ||
     std::abs(exact.raw.clearanceCostMeters-
              edt.raw.clearanceCostMeters)>1e-9)
    throw std::runtime_error("EDT changed clearance objective or search work");
  if(a.clearanceDistanceQueries!=a.clearanceExactFallbacks ||
     b.clearanceDistanceQueries!=b.clearanceEdtZeroCostBypasses+
                                 b.clearanceExactFallbacks)
    throw std::runtime_error("EDT clearance accounting is inconsistent");
}
void header() {
  std::cout<<"case_id,draw_index,model,geometry,variant,weight,radius_m,"
              "inflation_m,fast_empty_aabb,repeats,status,"
              "search_p50_ms,search_p95_ms,refine_p50_ms,total_p50_ms,"
              "expanded,generated,pruned_by_key,exact_collision_calls,"
              "search_motion_calls,clearance_samples,clearance_queries,"
              "edt_zero_cost_bypasses,exact_clearance_fallbacks,"
              "edt_build_ms,edt_storage_bytes,raw_length_m,raw_cost_m,"
              "raw_clearance_cost_m,raw_sections,min_clearance_m,"
              "p10_clearance_m,quality_poses,refine_status,"
              "refined_length_m,refined_cost_m\n";
}
void row(const pipeline::Config& cfg,const pipeline::Scenario& scenario,
         fe::Variant geometry,Variant variant,const PlanRequest& req,
         const pipeline::Measurement& m,const Quality& quality) {
  const auto& d=m.raw.diagnostics;
  const bool refined=m.refinement && pipeline::strictSuccess(m.refinement->path);
  std::cout<<std::fixed<<std::setprecision(6)
    <<scenario.id<<','<<scenario.drawIndex<<','<<cfg.model<<','
    <<fe::label(geometry)<<','<<label(variant)<<','
    <<req.options.clearanceWeight<<','
    <<req.options.clearanceInfluenceRadiusMeters<<','
    <<req.options.footprintInflationMeters<<','
    <<(req.options.useFastEmptyAabb?1:0)<<','
    <<cfg.repeats<<','<<pipeline::statusName(m.raw.status)<<','
    <<m.searchP50<<','<<m.searchP95<<','<<m.refineP50<<','<<m.totalP50<<','
    <<d.expanded<<','<<d.generated<<','<<d.prunedByKey<<','
    <<d.exactCollisionPoseCalls<<','<<d.searchMotionCalls<<','
    <<d.clearanceSamples<<','<<d.clearanceDistanceQueries<<','
    <<d.clearanceEdtZeroCostBypasses<<','<<d.clearanceExactFallbacks<<','
    <<d.clearanceEdtBuildMilliseconds<<','<<d.clearanceEdtBytes<<','
    <<m.raw.geometricLengthMeters<<','<<m.raw.weightedCostMeters<<','
    <<m.raw.clearanceCostMeters<<','<<m.raw.sections.size()<<','
    <<quality.minClearance<<','<<quality.p10Clearance<<','<<quality.poses<<','
    <<(refined?pipeline::refinerName(m.refinement->status):"NotRun")<<','
    <<(refined?m.refinement->path.geometricLengthMeters:0)<<','
    <<(refined?m.refinement->path.weightedCostMeters:0)<<'\n';
}
} // namespace

int main(int argc,char** argv) {
  try {
    double weight=.15,radius=.4;
    bool fastAabb=false;
    std::size_t replayLimit=0; // 0 means run every validated replay case
    std::vector<char*> forwarded{argv[0]};
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--clearance-weight"||arg=="--clearance-radius") {
        if(i+1>=argc)throw std::invalid_argument("Missing value for "+arg);
        const double v=positiveReal(argv[++i]);
        if(arg=="--clearance-weight")weight=v;else radius=v;
      } else if(arg=="--fast-empty-aabb")fastAabb=true;
      else if(arg=="--replay-limit") {
        if(i+1>=argc)throw std::invalid_argument("Missing value for --replay-limit");
        replayLimit=pipeline::integer(argv[++i]);
      } else forwarded.push_back(argv[i]);
    }
    const auto cfg=pipeline::parseArgs(
        static_cast<int>(forwarded.size()),forwarded.data());
    if((cfg.model!="differential"&&cfg.model!="mecanum")||
       cfg.connector!="native"||cfg.collectCollisionWork)
      throw std::invalid_argument("Use native differential/mecanum, no collision-work mode");
    const auto map=hybrid_astar::demo::loadRosMapYaml(cfg.yamlPath);
    const auto base=pipeline::baseRequest(cfg,map);
    auto scenarios=cfg.replayCsv.empty()?
        pipeline::discover(cfg,base):pipeline::replayScenarios(cfg,base);
    // Validate the entire pinned replay fixture against --cases first.
    // Only then limit execution to a deterministic subset for smoke tests.
    if(replayLimit) {
      if(cfg.replayCsv.empty())
        throw std::invalid_argument("--replay-limit requires --replay-csv");
      if(replayLimit>scenarios.size())
        throw std::invalid_argument("--replay-limit exceeds validated route count");
      scenarios.resize(replayLimit);
    }
    std::cerr<<"Nav2 clearance A/B/C: "<<scenarios.size()
             <<" routes; weight="<<weight<<", radius="<<radius
             <<", fastAABB="<<fastAabb
             <<"; all EDT construction is inside the timed search.\n";
    header();
    const std::array<Variant,3> variants{
        Variant::Off,Variant::Exact,Variant::Edt};
    for(const auto& scenario:scenarios) {
      const std::array<fe::Variant,2> geometries=scenario.id%2?
          std::array{fe::Variant::SmallInflated,fe::Variant::LargeRectangle}:
          std::array{fe::Variant::LargeRectangle,fe::Variant::SmallInflated};
      for(auto geometry:geometries) {
        auto req=base;
        req.start=scenario.endpoints.start;
        req.goal=scenario.endpoints.goal;
        req.footprint=geometry==fe::Variant::LargeRectangle?
            Footprint{-.35,.35,.25}:Footprint{-.25,.25,.15};
        req.options.footprintInflationMeters=
            geometry==fe::Variant::LargeRectangle?0.:.1;
        req.options.useEmptyRegionPrefixSum=true;
        req.options.useFastEmptyAabb=fastAabb;
        req.options.useInflatedFootprintBroadPhase=true;
        std::array<std::optional<pipeline::Measurement>,3> measurements;
        const std::size_t offset=(scenario.id+
            (geometry==fe::Variant::SmallInflated?1:0))%3;
        for(std::size_t j=0;j<3;++j) {
          const auto variant=variants[(offset+j)%3];
          const auto index=static_cast<std::size_t>(variant);
          auto configured=req;
          configured.options.clearanceWeight=
              variant==Variant::Off?0.:weight;
          configured.options.clearanceInfluenceRadiusMeters=radius;
          configured.options.useClearanceDistanceTransform=
              variant==Variant::Edt;
          measurements[index]=pipeline::measure(cfg,configured);
        }
        if(!measurements[0]||!measurements[1]||!measurements[2])
          throw std::runtime_error("Missing clearance experiment variant");
        // Pinned replay routes must succeed without the optional objective.
        // A positive clearance weight MUST actually exercise clearance
        // scoring. Invalid input or zero-work "Other" rows previously passed
        // the exact/EDT equivalence gate and gave misleading microsecond times.
        if(measurements[0]->raw.status!=PlanStatus::Success)
          throw std::runtime_error("Baseline warehouse replay did not succeed: case "+
                                   std::to_string(scenario.id));
        for(std::size_t i=1;i<3;++i) {
          const auto& planResult=measurements[i]->raw;
          if(planResult.status==PlanStatus::InvalidRequest ||
             planResult.status==PlanStatus::InvalidMap ||
             planResult.status==PlanStatus::InternalError ||
             planResult.status==PlanStatus::RejectedUnknownMap ||
             planResult.diagnostics.clearanceDistanceQueries==0)
            throw std::runtime_error(
                std::string("Clearance mode ")+label(variants[i])+
                " failed before evaluating the objective on case "+
                std::to_string(scenario.id)+": "+
                pipeline::statusName(planResult.status));
        }
        checkEquivalent(*measurements[1],*measurements[2]);
        for(std::size_t i=0;i<3;++i) {
          const auto variant=variants[i];
          req.options.clearanceWeight=
              variant==Variant::Off?0.:weight;
          req.options.clearanceInfluenceRadiusMeters=radius;
          req.options.useClearanceDistanceTransform=
              variant==Variant::Edt;
          const auto quality=assessRawPath(req,measurements[i]->raw);
          row(cfg,scenario,geometry,variant,req,*measurements[i],quality);
        }
        std::cerr<<"Case "<<scenario.id<<" "<<fe::label(geometry)
                 <<" search off/exact/edt "
                 <<measurements[0]->searchP50<<"/"
                 <<measurements[1]->searchP50<<"/"
                 <<measurements[2]->searchP50<<" ms; collision calls "
                 <<measurements[0]->raw.diagnostics.exactCollisionPoseCalls<<"/"
                 <<measurements[1]->raw.diagnostics.exactCollisionPoseCalls<<"/"
                 <<measurements[2]->raw.diagnostics.exactCollisionPoseCalls<<'\n';
      }
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Nav2 clearance experiment failed: "<<e.what()<<'\n';
    return 2;
  }
}
