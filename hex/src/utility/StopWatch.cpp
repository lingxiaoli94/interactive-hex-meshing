#include "StopWatch.h"

namespace hex {

void StopWatch::Tic() { start_time_ = Clock::now(); }

std::chrono::duration<double> StopWatch::Toc() {
  return Clock::now() - start_time_;
}

}  // namespace hex
