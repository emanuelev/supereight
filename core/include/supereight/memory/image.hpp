#pragma once

#include <supereight/shared/commons.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <cassert>
#include <memory>

namespace se {

template<typename T>
class ImageAccessor;

template<typename T, typename AllocatorT = Eigen::aligned_allocator<T>>
class Image {
public:
    using value_type = T;
    using size_type  = std::size_t;

    using accessor_type = ImageAccessor<value_type>;

    Image() = default;

    Image(const size_type width, const size_type height)
        : width_(width), height_(height) {
        assert(width_ > 0 && height_ > 0);
        data_ = allocator_.allocate(width_ * height_);
    }

    Image(const Eigen::Vector2i& size) : Image(size.x(), size.y()) {}
    Image(const size_type size) : Image(size, 1) {}

    Image(const Image& other) = delete;

    Image(Image&& other)
        : width_{other.width_}, height_{other.height_}, data_{other.data_} {
        other.width_  = 0;
        other.height_ = 0;
        other.data_   = nullptr;
    }

    ~Image() { allocator_.deallocate(data_, width_ * height_); };

    size_type width() const { return width_; };
    size_type height() const { return height_; };

    size_type size() const { return width_ * height_; };
    Eigen::Vector2i dim() const { return Eigen::Vector2i(width(), height()); }

    void resize(const size_type width, const size_type height) {
        if (width == width_ && height == height_) return;

        if (width * height == size()) {
            width_  = width;
            height_ = height;

            return;
        }

        allocator_.deallocate(data_, size());

        width_  = width;
        height_ = height;

        data_ = allocator_.allocate(size());
    }

    void resize(const size_type size) { resize(size, 1); }

    value_type* data() { return data_; }
    const value_type* data() const { return data_; }

    value_type& operator[](const size_type idx) { return data_[idx]; }
    const value_type& operator[](const size_type idx) const {
        return data_[idx];
    }

    value_type& operator()(const size_type x, const size_type y) {
        return data_[x + y * width_];
    }

    const value_type& operator()(const size_type x, const size_type y) const {
        return data_[x + y * width_];
    }

    accessor_type accessor() { return accessor_type(data_, width_, height_); }

    const accessor_type accessor() const {
        return accessor_type(data_, width_, height_);
    }

private:
    size_type width_  = 0;
    size_type height_ = 0;

    value_type* data_ = nullptr;

    AllocatorT allocator_;
};

template<typename T>
class ImageAccessor {
public:
    using value_type = T;
    using size_type  = std::size_t;

    ImageAccessor() = delete;

    SE_DEVICE_FUNC
    size_type width() const { return width_; };

    SE_DEVICE_FUNC
    size_type height() const { return height_; };

    SE_DEVICE_FUNC
    size_type size() const { return width_ * height_; };

    SE_DEVICE_FUNC
    Eigen::Vector2i dim() const { return Eigen::Vector2i(width(), height()); }

    SE_DEVICE_FUNC
    value_type* data() { return data_; }

    SE_DEVICE_FUNC
    const value_type* data() const { return data_; }

    SE_DEVICE_FUNC
    value_type& operator[](size_type idx) { return data_[idx]; }

    SE_DEVICE_FUNC
    const value_type& operator[](size_type idx) const { return data_[idx]; }

    SE_DEVICE_FUNC
    value_type& operator()(const size_type x, const size_type y) {
        return data_[x + y * width_];
    }

    SE_DEVICE_FUNC
    const value_type& operator()(const size_type x, const size_type y) const {
        return data_[x + y * width_];
    }

private:
    ImageAccessor(
        value_type* const data, const size_type width, const size_type height)
        : data_{data}, width_{width}, height_{height} {}

    ImageAccessor(const value_type* const data, const size_type width,
        const size_type height)
        : data_{const_cast<value_type*>(data)}, width_{width}, height_{height} {
    }

    value_type* const data_ = nullptr;

    size_type width_  = 0;
    size_type height_ = 0;

    template<typename, typename>
    friend class Image;
};

} // end namespace se
