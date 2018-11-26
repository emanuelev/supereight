#ifndef TIMINGS_H
#define TIMINGS_H

#include <cstdlib>
#include <chrono>

#define TICK() { \
  std::chrono::time_point<std::chrono::steady_clock> tickdata;\
    tickdata = std::chrono::steady_clock::now();

#define TOCK(str, size) \
  auto tockdata = std::chrono::steady_clock::now(); \
  auto diff = tockdata - tickdata;\
  Stats.sample(str, std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count()); \
}

static bool print_kernel_timing = false;
#endif //TIMINGS_H
