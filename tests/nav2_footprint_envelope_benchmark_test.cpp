// Manually-run full-resolution Nav2 experiment:
// A: 70x50 cm sharp-edged rectangle / 0 m inflation
// B: 50x30 cm rectangle / 0.10 m round-corner inflation
// Same map, same start/goal/yaw, robot model, planner/refiner settings.
#include "tests/nav2_footprint_envelope_benchmark_support.hpp"
#include <exception>
#include <iostream>
#include <string>
#include <vector>

int main(int argc,char** argv) {
  using namespace hybrid_astar::bench::footprint_envelope;
  try {
    std::size_t microPoses=20000,microRepeats=5;
    std::vector<char*> forwarded;
    forwarded.reserve(static_cast<std::size_t>(argc));
    forwarded.push_back(argv[0]);
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--micro-poses"||arg=="--micro-repeats") {
        if(i+1>=argc)
          throw std::invalid_argument("Missing value for "+arg);
        const auto n=pipeline::integer(argv[++i]);
        if(arg=="--micro-poses")microPoses=n;
        else microRepeats=n;
      }else forwarded.push_back(argv[i]);
    }
    if(microPoses<100||microPoses>1000000||microRepeats>100)
      throw std::invalid_argument("Use --micro-poses 100..1000000, --micro-repeats 1..100");
    const auto cfg=pipeline::parseArgs(
        static_cast<int>(forwarded.size()),forwarded.data());
    if(cfg.model!="differential"||cfg.connector!="native")
      throw std::invalid_argument("Envelope A/B is for differential-drive native planning only");
    if(cfg.collectCollisionWork)
      throw std::invalid_argument("Do not use --collect-collision-work; the experiment "
                                  "collects counters outside timed runs automatically");

    const auto map=hybrid_astar::demo::loadRosMapYaml(cfg.yamlPath);
    if(map->width!=1006||map->height!=1674||
       std::abs(map->resolutionMeters-0.03)>1e-12)
      throw std::invalid_argument("Expected unchanged 1006x1674 Nav2 warehouse at 0.03 m/cell");
    const auto baseline=pipeline::baseRequest(cfg,map);
    const auto cases=cfg.replayCsv.empty()?
        pipeline::discover(cfg,baseline):pipeline::replayScenarios(cfg,baseline);
    std::cerr<<"Footprint envelope A/B: "<<cases.size()
             <<" routes, "<<cfg.repeats<<" pipeline repetitions per geometry, "
             <<microPoses<<" fixed random micro poses x "<<microRepeats
             <<", grid cap "<<cfg.gridSettlements
             <<". Search/refinement and microtimes exclude collision-work counters.\n";
    printHeader();
    for(const auto& scenario:cases) {
      const auto bank=fixedPoseBank(cfg,scenario,*map,microPoses);
      // Alternate variant execution order by case to limit cache/frequency bias.
      const auto a=Variant::LargeRectangle,b=Variant::SmallInflated;
      const std::array<Variant,2> order=scenario.id%2?
          std::array<Variant,2>{b,a}:std::array<Variant,2>{a,b};
      std::optional<Result> large,small;
      for(const auto variant:order) {
        auto request=configure(baseline,variant);
        request.start=scenario.endpoints.start;
        request.goal=scenario.endpoints.goal;
        auto result=run(cfg,request,bank,microRepeats);
        if(variant==a)large=std::move(result);
        else small=std::move(result);
      }
      if(!large||!small)
        throw std::runtime_error("Missing complete paired geometry measurement");
      // Do NOT assert path equivalence: rounded vs sharp corners legitimately
      // change collision status and search route. Preserve those differences.
      auto reqA=configure(baseline,a),reqB=configure(baseline,b);
      reqA.start=reqB.start=scenario.endpoints.start;
      reqA.goal=reqB.goal=scenario.endpoints.goal;
      printRow(cfg,scenario,a,reqA,*large);
      printRow(cfg,scenario,b,reqB,*small);
      std::cerr<<"Case "<<scenario.id
        <<": 70x50 sharp "<<pipeline::statusName(large->pipeline.raw.status)
        <<" search "<<large->pipeline.searchP50<<"ms"
        <<", 50x30+0.10 rounded "<<pipeline::statusName(small->pipeline.raw.status)
        <<" search "<<small->pipeline.searchP50<<"ms"
        <<"; micro "<<large->micro.medianUsPerPose<<" vs "
        <<small->micro.medianUsPerPose<<" us/pose\n";
    }
    return 0;
  }catch(const std::exception& ex) {
    std::cerr<<"Nav2 equivalent-envelope footprint benchmark failed: "
             <<ex.what()<<'\n';
    return 2;
  }
}
