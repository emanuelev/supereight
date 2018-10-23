#ifndef IMAGE_H
#define IMAGE_H

#include <vector>
#include <cassert>

namespace se {

  template <typename T>
    class Image {

      public:
        Image(const unsigned w, const unsigned h) : width_(w), height_(h) {
          assert(width_ > 0 && height_ > 0);
          data.resize(width_ * height_);
        }

        Image(const unsigned w, const unsigned h, const T& val) : width_(w), height_(h) {
          assert(width_ > 0 && height_ > 0);
          data.resize(width_ * height_, val);
        }

        T&       operator[](std::size_t idx)       { return data[idx]; }
        const T& operator[](std::size_t idx) const { return data[idx]; }

        T&       operator()(const int x, const int y)       { return data[x + y*width_]; }
        const T& operator()(const int x, const int y) const { return data[x + y*width_]; }

        std::size_t size()   const   { return width_ * height_; };
        int         width () const { return width_;  };
        int         height() const { return height_; };

      private:
        const int width_;
        const int height_;
        std::vector<T> data;
    };

} // end namespace se
#endif
