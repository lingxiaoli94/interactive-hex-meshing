#pragma once

#include <chrono>

namespace hex {
class StopWatch {
 public:
  using Clock = std::chrono::steady_clock;

  void Tic();
  std::chrono::duration<double> Toc();

 private:
  std::chrono::time_point<Clock> start_time_;
};
}  // namespace hex
