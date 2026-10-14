#pragma once
#include <cstddef>

#include "proto.h"

namespace pk {

// Fixed-capacity FIFO of Samples. Single-context use (no locking).
template <size_t N>
class RingBuffer {
 public:
  // Returns false (and stores nothing) when full; caller counts the drop.
  bool push(const Sample& s) {
    if (count_ == N) return false;
    buf_[(head_ + count_) % N] = s;
    ++count_;
    return true;
  }

  // Pops up to `max` samples in FIFO order; returns number popped.
  size_t pop(Sample* out, size_t max) {
    size_t n = count_ < max ? count_ : max;
    for (size_t i = 0; i < n; ++i) {
      out[i] = buf_[head_];
      head_ = (head_ + 1) % N;
    }
    count_ -= n;
    return n;
  }

  // Copies up to `max` samples from the front without consuming them.
  // Pair with drop() to remove them only once they have been delivered.
  size_t peek(Sample* out, size_t max) const {
    size_t n = count_ < max ? count_ : max;
    for (size_t i = 0; i < n; ++i) out[i] = buf_[(head_ + i) % N];
    return n;
  }

  // Discards up to `n` samples from the front; returns number discarded.
  size_t drop(size_t n) {
    if (n > count_) n = count_;
    head_ = (head_ + n) % N;
    count_ -= n;
    return n;
  }

  size_t size() const { return count_; }

  void clear() {
    head_ = 0;
    count_ = 0;
  }

 private:
  Sample buf_[N];
  size_t head_ = 0;
  size_t count_ = 0;
};

}  // namespace pk
