#pragma once

#include <supereight/utils/cuda_util.hpp>

#include <cuda_runtime.h>

namespace se {

template<typename T>
class allocator_cuda {
public:
    using value_type      = T;
    using size_type       = std::size_t;
    using difference_type = std::ptrdiff_t;

    using propagate_on_container_move_assignment = std::true_type;
    using is_always_equal                        = std::true_type;

    template<class U>
    struct rebind {
        typedef allocator_cuda<U> other;
    };

    allocator_cuda() {}
    allocator_cuda(const allocator_cuda&) {}

    template<class U>
    allocator_cuda(const allocator_cuda<U>&) {}

    ~allocator_cuda() {}

    value_type* allocate(size_type num, const void* = 0) {
        size_type size = num * sizeof(T);

        value_type* p;
        safeCall(cudaMallocManaged(&p, size));

        return p;
    }

    void deallocate(value_type* p, size_type) { safeCall(cudaFree(p)); }
};

} // namespace se
