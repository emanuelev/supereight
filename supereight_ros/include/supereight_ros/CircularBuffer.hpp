#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP 1
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <mutex>
#include <Eigen/Dense>

template<class T>
class CircularBuffer {
public:
  CircularBuffer(size_t size) :
    buf_(std::unique_ptr<T[]>(new T[size])),
    max_size_(size) {
      // empty
  }

  void put(T item);
  void get(uint64_t time_stamp, T& pre_item, T& post_item);
  void reset();
  bool empty() const;
  bool full() const;
  size_t capacity() const;
  size_t size() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::mutex mutex_;
  std::unique_ptr<T[]> buf_;
  size_t head_ = 0;
  size_t tail_ = 0;
  const size_t max_size_;
  bool full_ = 0;
};

#endif //CIRCULAR_BUFFER_HPP

template<class T>
void CircularBuffer<T>::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  head_ = tail_;
  full_ = false;
}

template<class T>
bool CircularBuffer<T>::empty() const {
  return (!full_ && (head_ == tail_));
}

template<class T>
bool CircularBuffer<T>::full() const {
  return full_;
}

template<class T>
size_t CircularBuffer<T>::capacity() const {
  return max_size_;
}

template<class T>
size_t CircularBuffer<T>::size() const {
  size_t size = max_size_;

  if (!full_) {
    if (head_ >= tail_) {
      size = head_ - tail_;
    } else {
      size = max_size_ + head_ - tail_;
    }
  }

  return size;
}

template<class T>
void CircularBuffer<T>::put(T item) {

  std::lock_guard<std::mutex> lock(mutex_);

  buf_[head_] = item;

  if (full_) {
    tail_ = (tail_ + 1) % max_size_;
  }

  head_ = (head_ + 1) % max_size_;

  full_ = head_ == tail_;
}

template<class T>
void CircularBuffer<T>::get(uint64_t time_stamp, T& pre_item, T& post_item) {

  std::lock_guard<std::mutex> lock(mutex_);

  while(time_stamp > ros::Time(buf_[(tail_ + 1) % max_size_].header.stamp).toNSec())
    tail_ = (tail_ + 1) % max_size_;

  size_t post_tail((tail_ + 1) % max_size_);
  pre_item = buf_[tail_];
  post_item = buf_[post_tail];

  if (full_ && (tail_ != head_))
    full_ = false;
}

template class CircularBuffer<geometry_msgs::TransformStamped>; 