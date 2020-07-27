#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

namespace se {
namespace tracking {

struct TrackData {
    int result;
    float error;
    float J[6];
};

inline void compareTrackData(
    std::string str, TrackData* l, TrackData* r, unsigned int size) {
    const float epsilon = 0.0000001;

    for (unsigned int i = 0; i < size; i++) {
        if (std::abs(l[i].error - r[i].error) > epsilon) {
            std::cout << "Error into " << str << " at " << i << std::endl;
            std::cout << "l.error =  " << l[i].error << std::endl;
            std::cout << "r.error =  " << r[i].error << std::endl;
        }

        if (std::abs(l[i].result - r[i].result) > epsilon) {
            std::cout << "Error into " << str << " at " << i << std::endl;
            std::cout << "l.result =  " << l[i].result << std::endl;
            std::cout << "r.result =  " << r[i].result << std::endl;
        }
    }
}

} // namespace tracking
} // namespace se
