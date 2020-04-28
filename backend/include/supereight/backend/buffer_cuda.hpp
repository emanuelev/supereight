#pragma once

#include <supereight/backend/cuda_util.hpp>
#include <supereight/shared/commons.h>

#include <cuda_runtime.h>

namespace se {

template<typename T>
class BufferAccessorCUDA;

template<typename T>
class BufferCUDA {
public:
    SE_DEVICE_FUNC
    BufferCUDA(){};

    SE_DEVICE_FUNC
    BufferCUDA(int size) : size_{size} {
        safeCall(cudaMallocManaged(&buffer_, size_ * sizeof(T)));
    };

    SE_DEVICE_FUNC
    BufferCUDA(const BufferCUDA&& other)
        : buffer_{other.buffer_}, size_{other.size_} {}

    BufferCUDA(const BufferCUDA&) = delete;

    SE_DEVICE_FUNC
    ~BufferCUDA() { safeCall(cudaFree(buffer_)); }

    SE_DEVICE_FUNC
    BufferAccessorCUDA<T> accessor() {
        return BufferAccessorCUDA<T>(buffer_, size_);
    }

    SE_DEVICE_FUNC
    void resize(int n) {
        if (size_ == n) return;

        size_ = n;

        safeCall(cudaFree(buffer_));
        safeCall(cudaMallocManaged(&buffer_, size_ * sizeof(T)));
    }

    SE_DEVICE_FUNC
    int size() const { return size_; }

private:
    T* buffer_ = nullptr;
    int size_  = 0;
};

template<typename T>
class BufferAccessorCUDA {
public:
    BufferAccessorCUDA() = delete;

    SE_DEVICE_FUNC
    int size() const { return size_; }

    SE_DEVICE_FUNC
    T* data() const { return buffer_; }

    SE_DEVICE_FUNC
    T* operator[](int i) const { return buffer_ + i; }

private:
    SE_DEVICE_FUNC
    BufferAccessorCUDA(T* buffer, int size) : buffer_{buffer}, size_{size} {}

    T* buffer_;
    int size_;

    friend class BufferCUDA<T>;
};

} // namespace se
