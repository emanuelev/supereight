#pragma once

#include <atomic>
#include <cstdlib>

namespace se {

template<typename T>
class MemoryPool {
public:
    ~MemoryPool() {
        for (auto page : pages_) delete page;
    }

    std::size_t used() const { return used_; }

    std::size_t capacity() const { return capacity_; }

    void clear() { used_ = 0; }

    T* acquire() {
        std::size_t idx = used_.fetch_add(1);
        return (*this)[idx];
    }

    void reserve(std::size_t n) {
        if (n <= capacity()) return;

        std::size_t num_pages = (n - capacity() + page_size_ - 1) / page_size_;
        for (std::size_t i = 0; i < num_pages; ++i) {
            pages_.push_back(new T[page_size_]);
            capacity_ += page_size_;
        }
    }

    T* operator[](std::size_t i) const {
        std::size_t page   = i / page_size_;
        std::size_t offset = i % page_size_;

        return pages_[page] + offset;
    }

private:
    static constexpr std::size_t page_size_ = 8 * 4096;

    std::atomic<std::size_t> used_ = {0};
    std::size_t capacity_          = 0;

    std::vector<T*> pages_;
};

} // namespace se
