#pragma once

#include <supereight/memory/allocator_cuda.hpp>
#include <supereight/memory/image.hpp>

namespace se {

template<typename T>
using ImageCUDA = Image<T, allocator_cuda<T>>;

template<typename T>
using ImageAccessorCUDA = ImageAccessor<T>;

} // namespace se
