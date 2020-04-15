#pragma once

#include <supereight/index_traits.hpp>

#include <atomic>
#include <cstdlib>

namespace se {

template<>
struct index_traits<int> {
    static inline int invalid() { return -1; }
}

template<typename T>
class BufferCPU {
public:
    using index_t = int;

    BufferCPU(int cap) {
        buffer_ = std::calloc(cap, sizeof(T));
        used_   = new std::atomic<int>(0);

        capacity_ = cap;
    };

    ~BufferCPU() { std::free(buffer_); };

    int used() { return *used_; };
    int capacity() { return capacity_; };

    index_t acquire() { return used_->fetch_add(1); }

    void reserve(int n) {
        if (n <= capacity()) return;

        std::realloc(buffer_, n * sizeof(T));
        capacity_ = n;
    };

    index_t first() { return 0; };
    index_t last() { return static_cast<index_t>(used()); };

    T& operator[](index_t i) { return buffer_[i]; };
    T operator[](index_t i) const { return buffer_[i]; };

private:
    std::atomic<int>* used_;
    int capacity_;

    T* buffer_;
};

} // namespace se
