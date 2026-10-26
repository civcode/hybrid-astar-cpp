#include <catch2/catch_test_macros.hpp>
#include "hybrid_astar/ackermann/open_queue.hpp"
#include "hybrid_astar/ackermann/planner.hpp"
#include "bench/hard_scenarios.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <random>
#include <vector>
using namespace hybrid_astar::ackermann;

TEST_CASE("Binary and indexed 4-ary frontiers preserve priority and replace active labels",
          "[w6][queue]") {
  for(auto mode:{OpenQueueMode::BinaryLazy,OpenQueueMode::FourAryLazy,
                 OpenQueueMode::IndexedBinary,OpenQueueMode::IndexedFourAry}) {
    DYNAMIC_SECTION(static_cast<int>(mode)) {
      SearchOpenQueue q(mode);
      std::map<int,OpenEntry> active;
      std::map<int,std::size_t> previous;
      std::vector<int> labels;
      std::mt19937 rng(20261026);
      std::uniform_real_distribution<double> cost(.1,150.);
      std::size_t replaced=0;
      for(int i=0;i<4500;++i) {
        if(i%7==0&&!active.empty()) {
          const auto best=std::min_element(active.begin(),active.end(),[](const auto& a,const auto& b){
            return OpenEntryWorse{}(b.second,a.second);
          });
          OpenEntry item{};bool live=false;
          while(!q.empty()) {
            item=q.pop();const auto label=labels.at(item.node);
            if(active.contains(label)&&active.at(label).node==item.node){live=true;break;}
          }
          REQUIRE(live);
          REQUIRE(item.node==best->second.node);
          active.erase(best);
        }else {
          const int label=int(rng()%200);
          const OpenEntry value{i%11==0?20.:cost(rng),i%11==0?3.:cost(rng),labels.size()};
          std::optional<std::size_t> old;
          if(previous.contains(label))old=previous.at(label);
          labels.push_back(label);
          if(q.push(value,old))++replaced;
          active[label]=value;previous[label]=value.node;
        }
      }
      while(!active.empty()) {
        const auto best=std::min_element(active.begin(),active.end(),[](const auto& a,const auto& b){
          return OpenEntryWorse{}(b.second,a.second);
        });
        OpenEntry item{};bool live=false;
        while(!q.empty()) {
          item=q.pop();const auto label=labels.at(item.node);
          if(active.contains(label)&&active.at(label).node==item.node){live=true;break;}
        }
        REQUIRE(live);
        REQUIRE(item.node==best->second.node);
        active.erase(best);
      }
      if(mode==OpenQueueMode::IndexedBinary||mode==OpenQueueMode::IndexedFourAry)
        REQUIRE(replaced>100);
      else REQUIRE(replaced==0);
    }
  }
}

TEST_CASE("Heap tuning preserves strictly validated hard-scenario results",
          "[w6][queue][hard-scenarios]") {
  for(const auto& item:hybrid_astar::bench::hardCases()) {
    DYNAMIC_SECTION(item.name) {
      auto request=item.request;
      request.options.heuristicMode=HeuristicMode::PerformanceReedsSheppAndObstacleGrid;
      request.options.performanceHeuristicWeight=1.2;
      request.options.gridMaxTotalSettlements=20000;
      const auto baseline=plan(request);
      for(auto mode:{OpenQueueMode::BinaryLazy,OpenQueueMode::FourAryLazy,
                     OpenQueueMode::IndexedBinary,OpenQueueMode::IndexedFourAry}) {
        for(bool tuneGrid:{false,true}) {
          request.options.openQueueMode=tuneGrid?OpenQueueMode::BinaryLazy:mode;
          request.options.gridQueueMode=tuneGrid?mode:OpenQueueMode::BinaryLazy;
          const auto actual=plan(request);
          INFO("mode "<<static_cast<int>(mode)<<" grid="<<tuneGrid);
          REQUIRE(actual.status==baseline.status);
          REQUIRE(actual.diagnostics.expanded==baseline.diagnostics.expanded);
          REQUIRE(actual.diagnostics.generated==baseline.diagnostics.generated);
          REQUIRE(actual.diagnostics.gridSettlements==baseline.diagnostics.gridSettlements);
          REQUIRE(actual.sections.size()==baseline.sections.size());
          REQUIRE(std::abs(actual.weightedCostMeters-baseline.weightedCostMeters)<1e-9);
          REQUIRE(actual.collisionEvidence==baseline.collisionEvidence);
          REQUIRE(actual.mapRevision==baseline.mapRevision);
          REQUIRE(actual.diagnostics.openQueuePushes==actual.diagnostics.generated);
          REQUIRE(actual.diagnostics.openQueuePops>=actual.diagnostics.expanded);
          REQUIRE(actual.diagnostics.gridQueuePops>=actual.diagnostics.gridSettlements);
          REQUIRE(actual.diagnostics.openQueuePeakStorageBytes>0);
          if(actual.status==PlanStatus::Success)
            REQUIRE(actual.collisionEvidence==CollisionEvidence::FullFootprintBodyMotionAdaptiveSampled);
          if(mode==OpenQueueMode::IndexedBinary||mode==OpenQueueMode::IndexedFourAry) {
            if(tuneGrid)REQUIRE(actual.diagnostics.gridQueueStalePops==0);
            else REQUIRE(actual.diagnostics.staleHeapPops==0);
          }
        }
      }
    }
  }
}
TEST_CASE("Invalid priority queue modes do not silently select another algorithm",
          "[w6][queue]") {
  auto request=hybrid_astar::bench::hardCases().front().request;
  request.options.openQueueMode=static_cast<OpenQueueMode>(123);
  REQUIRE(plan(request).status==PlanStatus::InvalidRequest);
  request.options.openQueueMode=OpenQueueMode::BinaryLazy;
  request.options.gridQueueMode=static_cast<OpenQueueMode>(123);
  REQUIRE(plan(request).status==PlanStatus::InvalidRequest);
}
