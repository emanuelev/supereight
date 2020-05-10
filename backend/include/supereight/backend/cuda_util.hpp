#pragma once

#include <cuda_runtime.h>

#define safeCall(expr) safeCall_(expr, #expr, __FILE__, __LINE__)
inline void safeCall_(
    cudaError_t code, const char* expr, const char* file, int line) {
    if (code != cudaSuccess) {
        std::cout << "CUDA driver API error: code " << code << " - "
                  << cudaGetErrorString(code) << ", expr " << expr << ", file "
                  << file << ", line " << line << std::endl;

        exit(-1);
    }
}
