#pragma once

#include <supereight/backend/cuda_util.hpp>
#include <supereight/shared/commons.h>

#include <cuda_runtime.h>

namespace se {

template<typename T>
class MemoryPoolCUDA {
public:
    SE_DEVICE_FUNC
    MemoryPoolCUDA() {
        safeCall(cudaMallocManaged(&used_, sizeof(int)));
        *used_ = 0;
    };

    SE_DEVICE_FUNC
    MemoryPoolCUDA(const MemoryPoolCUDA&) = default;

    SE_DEVICE_FUNC
    ~MemoryPoolCUDA() {}

    int used() const {
        int used;
        cudaMemcpy(&used, used_, sizeof(int), cudaMemcpyDeviceToHost);

        return used;
    }

    int capacity() const { return capacity_; }

    SE_DEVICE_FUNC
    void clear() { *used_ = 0; }

    SE_DEVICE_ONLY_FUNC
    T* acquire() {
#ifdef __CUDACC__
        int idx = atomicAdd(used_, 1);
#else
        int idx = (*used_)++;
#endif

        return (*this)[idx];
    }

    void reserve(int n) {
        if (n <= capacity_) return;
        int num_pages = (n - capacity() + page_size_ - 1) / page_size_;

        if (page_table_capacity_ < page_table_used_ + num_pages) {
            int new_capacity = (page_table_used_ + num_pages) * 1.5;

            T** new_table;
            safeCall(cudaMallocManaged(&new_table, new_capacity * sizeof(T*)));

            if (page_table_ != nullptr) {
                safeCall(cudaMemcpy(new_table, page_table_,
                    page_table_capacity_ * sizeof(T*),
                    cudaMemcpyDeviceToDevice));
                safeCall(cudaFree(page_table_));
            }

            page_table_          = new_table;
            page_table_capacity_ = new_capacity;
        }

        for (int i = 0; i < num_pages; ++i) {
            T* new_page;
            safeCall(cudaMallocManaged(&new_page, page_size_ * sizeof(T)));

            safeCall(cudaMemcpy(page_table_ + page_table_used_, &new_page,
                sizeof(T*), cudaMemcpyHostToDevice));
            page_table_used_++;

            capacity_ += page_size_;
        }
    }

    SE_DEVICE_FUNC
    T* operator[](int i) const {
        int page   = i / page_size_;
        int offset = i % page_size_;

        return page_table_[page] + offset;
    }

private:
    static constexpr int page_size_ = 32 * 4096 / sizeof(T);

    int capacity_ = 0;
    int* used_    = nullptr;

    T** page_table_ = nullptr;

    int page_table_used_     = 0;
    int page_table_capacity_ = 0;
};

} // namespace se
