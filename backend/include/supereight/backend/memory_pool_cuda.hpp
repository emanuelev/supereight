#pragma once

#include <cuda_runtime.h>

#include <atomic>
#include <cstdlib>
#include <vector>

namespace se {

template<typename T>
class MemoryPoolCUDA {
public:
    ~MemoryPoolCUDA() {
        for (auto page : pages_) std::free(page);
    }

    int used() const { return used_; }

    int capacity() const { return capacity_; }

    void clear() { used_ = 0; }

    T* acquire() {
        int idx = used_.fetch_add(1);
        return (*this)[idx];
    }

    void reserve(int n) {
        if (n <= capacity()) return;

        int num_pages = (n - capacity() + page_size_ - 1) / page_size_;
        for (int i = 0; i < num_pages; ++i) {
            T* page;
            cudaMallocManaged(&page, sizeof(T) * page_size_);

            pages_.push_back(page);
            capacity_ += page_size_;
        }
    }

    T* operator[](int i) const {
        int page   = i / page_size_;
        int offset = i % page_size_;

        return pages_[page] + offset;
    }

private:
    static constexpr int page_size_ = 4 * 4096 / sizeof(T);

    std::atomic<int> used_ = 0;
    int capacity_          = 0;

    std::vector<T*> pages_;
};

} // namespace se