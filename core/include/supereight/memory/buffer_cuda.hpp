#pragma once

#include <supereight/utils/cuda_util.hpp>
#include <supereight/shared/commons.h>

#include <cuda_runtime.h>

namespace se {

template<typename T>
class BufferAccessorCUDA;

template<typename T>
class BufferCUDA {
public:
    BufferCUDA(){};

    BufferCUDA(std::size_t size) : size_{size} {
        safeCall(cudaMallocManaged(&buffer_, size_ * sizeof(T)));
    };

    BufferCUDA(const BufferCUDA&& other)
        : buffer_{other.buffer_}, size_{other.size_} {}

    BufferCUDA(const BufferCUDA&) = delete;

    ~BufferCUDA() { safeCall(cudaFree(buffer_)); }

    BufferAccessorCUDA<T> accessor() {
        return BufferAccessorCUDA<T>(buffer_, size_);
    }

    void resize(std::size_t n) {
        if (size_ == n) return;

        size_ = n;

        safeCall(cudaFree(buffer_));
        safeCall(cudaMallocManaged(&buffer_, size_ * sizeof(T)));
    }

    void fill(std::uint8_t val) {
        if (buffer_ == nullptr) return;
        cudaMemset(buffer_, val, size_ * sizeof(T));
    }

    std::size_t size() const { return size_; }

private:
    T* buffer_        = nullptr;
    std::size_t size_ = 0;
};

template<typename T>
class BufferAccessorCUDA {
public:
    BufferAccessorCUDA() = delete;

    SE_DEVICE_FUNC
    std::size_t size() const { return size_; }

    SE_DEVICE_FUNC
    T* data() const { return buffer_; }

    SE_DEVICE_FUNC
    const T& operator[](std::size_t i) const { return buffer_[i]; }

    SE_DEVICE_FUNC
    T& operator[](std::size_t i) { return buffer_[i]; }

private:
    SE_DEVICE_FUNC
    BufferAccessorCUDA(T* buffer, std::size_t size)
        : buffer_{buffer}, size_{size} {}

    T* buffer_;
    std::size_t size_;

    friend class BufferCUDA<T>;
};

} // namespace se
