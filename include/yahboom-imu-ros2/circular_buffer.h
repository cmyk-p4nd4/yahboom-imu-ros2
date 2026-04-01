#pragma once

#include <memory>
#include <mutex>
#include <initializer_list>
#include <stdexcept>

template <typename T, size_t N>
class circular_buffer {
public:
  static_assert(N > 0, "size of the Circular buffer must be greater than 0!");

  static const ssize_t npos = -1;

  // default constructor
  circular_buffer()
    : buffer_ptr_(std::unique_ptr<T[]>(new T[N])),
      write_idx_(0),
      read_idx_(0),
      full_(false) {
  }

  ~circular_buffer() {
    buffer_ptr_.reset();
    write_idx_ = 0;
    read_idx_ = 0;
  }

  void push_back(const T& value) {
    buffer_ptr_[write_idx_] = value;
    this->advance();
  }

  void push_back(T&& value) {
    buffer_ptr_[write_idx_] = std::move(value);
    this->advance();
  }

  void push_back(std::initializer_list<T> list) {
    for (auto it = list.begin(); it != list.end(); it++) {
      this->push_back(*it);
    }
  }

  template <class InputIt>
  void push_back(InputIt first, InputIt last) {
    while (first != last) {
      this->push_back(*first++);
    }
  }

  T pop_front() {
    if (this->empty()) {
      return T();
    }

    auto val = std::move(buffer_ptr_[read_idx_]);
    this->retreat();
    return val;
  }

  const T& front(void) {
    if (this->empty()) throw std::out_of_range("Buffer is empty");

    T& val = buffer_ptr_[read_idx_];
    return val;
  }

  // discard up-to `n` elemets (default 1 elements)
  // If there is less elements avaliable,  all elements will be removed
  void discard_n(size_t n = 1) {
    if (this->size() < n) {  // stored element < number of discarding element
      this->reset();         // discard all
    } else {
      while (n--) {
        this->retreat();
      }
    }
  }

  // reset the whole buffer
  void reset(void) {
    std::lock_guard<std::mutex> lock(mutex_);
    write_idx_ = read_idx_;
    full_ = false;
  }

  bool empty(void) const {
    return (!full_ && (write_idx_ == read_idx_));
  }

  bool full(void) const {
    return full_;
  }

  constexpr size_t capacity() {
    return this->max_size_;
  }

  size_t size() const {
    size_t size = !full_ ? 0 : max_size_;

    if (!full_) {
      if (write_idx_ >= read_idx_) {
        size = write_idx_ - read_idx_;
      } else {
        size = max_size_ + write_idx_ - read_idx_;
      }
    }

    return size;
  }

  ssize_t find(const T& needle) {
    size_t curr = read_idx_;
    size_t pos = 0;  // track how much have moved
    if (this->full_) {
      if (this->front() == needle) {
        return pos;
      } else {
        if (++curr == this->capacity()) {
          curr = 0;
        }
        pos++;
      }
    }
    while (curr != write_idx_) {
      if (buffer_ptr_[curr] == needle) {
        return pos;
      }
      if (++curr == this->capacity()) {
        curr = 0;
      }
      pos++;
    }

    return npos;
  }

protected:
  // Advance the write index by 1
  // if the buffer is full, discard the oldest one
  void advance(void) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (full_) {
      if (++read_idx_ == this->capacity()) {
        read_idx_ = 0;
      }
    }

    if (++write_idx_ == this->capacity()) {
      write_idx_ = 0;
    }

    full_ = write_idx_ == read_idx_;
  }

  // advance the read index by 1
  // if the buffer is empty, an exception is thrown
  void retreat(void) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (this->empty()) {
      throw std::out_of_range("Buffer is empty!");
    }

    if (++read_idx_ == this->capacity()) {
      read_idx_ = 0;
    }

    full_ = false;
  }

private:
  circular_buffer(circular_buffer&& other) = delete;

  std::mutex mutex_;
  std::unique_ptr<T[]> buffer_ptr_;
  size_t write_idx_ = 0;
  size_t read_idx_ = 0;
  const size_t max_size_ = N;
  bool full_ = false;
};
