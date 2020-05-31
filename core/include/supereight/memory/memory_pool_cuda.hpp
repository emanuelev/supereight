#pragma once

#include <supereight/shared/commons.h>
#include <supereight/utils/cuda_util.hpp>

#include <cuda_runtime.h>

namespace se {

#ifdef __CUDACC__
template<typename T, unsigned S>
__global__ void initPage(T* page) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= S) return;

    page[idx] = T();
}
#endif

template<typename T>
class MemoryPoolCUDA {
public:
    SE_DEVICE_FUNC
    MemoryPoolCUDA() {
        safeCall(cudaMallocManaged(&used_, sizeof(std::size_t)));
        *used_ = 0;
    };

    SE_DEVICE_FUNC
    MemoryPoolCUDA(const MemoryPoolCUDA&) = default;

    SE_DEVICE_FUNC
    ~MemoryPoolCUDA() {}

    std::size_t used() const {
        std::size_t used;
        cudaMemcpy(&used, used_, sizeof(std::size_t), cudaMemcpyDeviceToHost);

        return used;
    }

    std::size_t capacity() const { return capacity_; }

    SE_DEVICE_FUNC
    void clear() { *used_ = 0; }

    SE_DEVICE_ONLY_FUNC
    T* acquire() {
#ifdef __CUDACC__
        static_assert(sizeof(std::size_t) == sizeof(long long unsigned));
        std::size_t idx =
            atomicAdd(reinterpret_cast<long long unsigned*>(used_), 1);
#else
        std::size_t idx = (*used_)++;
#endif

        return (*this)[idx];
    }

    void reserve(std::size_t n) {
        if (n <= capacity_) return;
        std::size_t num_pages = (n - capacity() + page_size_ - 1) / page_size_;

        if (page_table_capacity_ < page_table_used_ + num_pages) {
            std::size_t new_capacity = (page_table_used_ + num_pages) * 4;

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

        for (std::size_t i = 0; i < num_pages; ++i) {
            T* new_page;
            safeCall(cudaMallocManaged(&new_page, page_size_ * sizeof(T)));

#ifdef __CUDACC__
            initPage<T, page_size_>
                <<<(page_size_ + 255) / 256, 256>>>(new_page);
#else
            std::fprintf(
                stderr, "warning: CPU init for CUDA memory pool page\n");
            for (unsigned i = 0; i < page_size_; ++i) { new_page[i] = T(); }
#endif

            safeCall(cudaMemcpy(page_table_ + page_table_used_, &new_page,
                sizeof(T*), cudaMemcpyHostToDevice));
            page_table_used_++;

            capacity_ += page_size_;
        }
    }

    SE_DEVICE_FUNC
    T* operator[](std::size_t i) const {
        std::size_t page   = i / page_size_;
        std::size_t offset = i % page_size_;

        return page_table_[page] + offset;
    }

private:
    static constexpr std::size_t page_size_ = 8 * 4096;

    std::size_t capacity_ = 0;
    std::size_t* used_    = nullptr;

    T** page_table_ = nullptr;

    std::size_t page_table_used_     = 0;
    std::size_t page_table_capacity_ = 0;
};

} // namespace se
