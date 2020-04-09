#ifndef TIMINGS_H
#define TIMINGS_H

#include <chrono>
#include <cstdlib>

#define TICK()                                                       \
    {                                                                \
    std::chrono::time_point<std::chrono::steady_clock> tickdata;     \
    tickdata = std::chrono::steady_clock::now();

#define TOCK(str, size)                                                      \
    auto tockdata = std::chrono::steady_clock::now();                        \
    auto diff     = tockdata - tickdata;                                     \
    Stats.sample(str,                                                        \
        std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count()); \
    }

#endif // TIMINGS_H
