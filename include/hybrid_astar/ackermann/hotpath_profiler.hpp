#pragma once
#include <cstdint>
#if defined(HYBRID_ASTAR_PROFILE_HOTPATHS)
#include <chrono>
#endif

namespace hybrid_astar::ackermann {
// Profile-only monotonic wall-clock accumulation. Destruct before result
// copies/returns so the reported counters contain completed measurements.
// Disabled builds incur no clock reads and no dynamic allocations.
class HotPathTimer final {
public:
#if defined(HYBRID_ASTAR_PROFILE_HOTPATHS)
  explicit HotPathTimer(std::uint64_t& total) noexcept
    : total_(&total), start_(std::chrono::steady_clock::now()) {}
  ~HotPathTimer() noexcept {
    using namespace std::chrono;
    *total_ += static_cast<std::uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now()-start_).count());
  }
private:
  std::uint64_t* total_;
  std::chrono::steady_clock::time_point start_;
#else
  explicit HotPathTimer(std::uint64_t&) noexcept {}
#endif
  HotPathTimer(const HotPathTimer&)=delete;
  HotPathTimer& operator=(const HotPathTimer&)=delete;
};
} // namespace hybrid_astar::ackermann
