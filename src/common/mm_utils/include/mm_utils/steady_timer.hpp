#pragma once
#include <chrono>

namespace mm_utils {
using SteadyClock = std::chrono::steady_clock;
inline double elapsedMilliseconds(SteadyClock::time_point start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

// Used for phase profiling on every return path, independently of /clock.
class ScopedWallTimer {
public:
  explicit ScopedWallTimer(double &total) : total_(total), start_(SteadyClock::now()) {}
  ~ScopedWallTimer() { total_ += elapsedMilliseconds(start_); }
  ScopedWallTimer(const ScopedWallTimer &) = delete;
  ScopedWallTimer &operator=(const ScopedWallTimer &) = delete;
private:
  double &total_;
  SteadyClock::time_point start_;
};
} // namespace mm_utils
