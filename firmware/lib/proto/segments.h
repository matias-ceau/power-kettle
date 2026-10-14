#pragma once
#include <cstddef>
#include <cstdint>

namespace pk {

// Tracks the sample indices held in a RingBuffer as runs of consecutive
// indices. When the ring is full, new samples are dropped, so the ring can
// contain gaps; an IMU packet must never span a gap or its `first_index`
// would be wrong. Each segment is one contiguous run, oldest first.
// At most K segments are tracked; when all are in use, a sample that would
// start a new segment must be dropped too (see can_append).
template <size_t K>
class Segments {
 public:
  bool empty() const { return n_ == 0; }

  // True if `index` extends the newest segment or a free slot exists.
  bool can_append(uint32_t index) const { return extends_back(index) || n_ < K; }

  // Records one sample; caller must have checked can_append().
  void append(uint32_t index) {
    if (extends_back(index)) {
      ++seg_[back()].count;
      return;
    }
    seg_[(head_ + n_) % K] = Segment{index, 1};
    ++n_;
  }

  uint32_t front_first() const { return n_ ? seg_[head_].first : 0; }
  size_t front_count() const { return n_ ? seg_[head_].count : 0; }

  // Removes `n` samples (n <= front_count()) from the oldest segment.
  void consume(size_t n) {
    if (n_ == 0) return;
    Segment& f = seg_[head_];
    if (n > f.count) n = f.count;
    f.first += static_cast<uint32_t>(n);
    f.count -= n;
    if (f.count == 0) {
      head_ = (head_ + 1) % K;
      --n_;
    }
  }

  void clear() {
    head_ = 0;
    n_ = 0;
  }

 private:
  struct Segment {
    uint32_t first;
    size_t count;
  };

  size_t back() const { return (head_ + n_ - 1) % K; }
  bool extends_back(uint32_t index) const {
    if (n_ == 0) return false;
    const Segment& b = seg_[back()];
    return b.first + static_cast<uint32_t>(b.count) == index;
  }

  Segment seg_[K];
  size_t head_ = 0;
  size_t n_ = 0;
};

}  // namespace pk
