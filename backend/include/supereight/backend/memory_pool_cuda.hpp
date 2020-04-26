#pragma once

#include <supereight/shared/commons.h>

#include <cuda_runtime.h>

namespace se {

template<typename T>
class MemoryPoolCUDA {
public:
    SE_DEVICE_FUNC
    MemoryPoolCUDA() { cudaMallocManaged(&buffer_, capacity_ * sizeof(T)); };

    SE_DEVICE_FUNC
    MemoryPoolCUDA(const MemoryPoolCUDA& other)
        : used_{other.used_}, buffer_{other.buffer_} {}

    SE_DEVICE_FUNC
    ~MemoryPoolCUDA() {}

    int used() const { return used_; }

    int capacity() const { return capacity_; }

    void clear() { used_ = 0; }

    SE_DEVICE_FUNC
    T* acquire() {
        int idx = used_++;
        return (*this)[idx];
    }

    void reserve(int) {}

    SE_DEVICE_FUNC
    T* operator[](int i) const { return buffer_ + i; }

private:
    constexpr static int capacity_ = 36728;
    int used_                      = 0;

    T* buffer_;
};

} // namespace se
