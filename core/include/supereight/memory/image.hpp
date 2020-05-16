#ifndef IMAGE_H
#define IMAGE_H

#include <cassert>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace se {

template<typename T>
class ImageAccessor;

template<typename T>
class Image {
public:
    using accessor_type = ImageAccessor<T>;

    Image(const Eigen::Vector2i& size) : Image(size.x(), size.y()) {}

    Image(const unsigned w, const unsigned h) : width_(w), height_(h) {
        assert(width_ > 0 && height_ > 0);
        data_.resize(width_ * height_);
    }

    Image(const unsigned w, const unsigned h, const T& val)
        : width_(w), height_(h) {
        assert(width_ > 0 && height_ > 0);
        data_.resize(width_ * height_, val);
    }

    int width() const { return width_; };
    int height() const { return height_; };

    std::size_t size() const { return width_ * height_; };
    Eigen::Vector2i dim() const { return Eigen::Vector2i(width(), height()); }

    T* data() { return data_.data(); }
    const T* data() const { return data_.data(); }

    T& operator[](std::size_t idx) { return data_[idx]; }
    const T& operator[](std::size_t idx) const { return data_[idx]; }

    T& operator()(const int x, const int y) { return data_[x + y * width_]; }
    const T& operator()(const int x, const int y) const {
        return data_[x + y * width_];
    }

    accessor_type accessor() {
        return accessor_type(data_.data(), width_, height_);
    }

    const accessor_type accessor() const {
        return accessor_type(data_.data(), width_, height_);
    }

private:
    int width_;
    int height_;

    std::vector<T, Eigen::aligned_allocator<T>> data_;
};

template<typename T>
class ImageAccessor {
public:
    ImageAccessor() = delete;

    int width() const { return width_; };
    int height() const { return height_; };

    std::size_t size() const { return width_ * height_; };
    Eigen::Vector2i dim() const { return Eigen::Vector2i(width(), height()); }

    T* data() { return data_; }
    const T* data() const { return data_; }

    T& operator[](std::size_t idx) { return data_[idx]; }
    const T& operator[](std::size_t idx) const { return data_[idx]; }

    T& operator()(const int x, const int y) { return data_[x + y * width_]; }
    const T& operator()(const int x, const int y) const {
        return data_[x + y * width_];
    }

private:
    ImageAccessor(T* data, int width, int height)
        : data_{data}, width_{width}, height_{height} {}

    ImageAccessor(const T* data, int width, int height)
        : data_{const_cast<T*>(data)}, width_{width}, height_{height} {}

    T* const data_;

    const int width_;
    const int height_;

    friend class Image<T>;
};

} // end namespace se
#endif
