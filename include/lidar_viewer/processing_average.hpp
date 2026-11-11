#pragma once
#include <array>
#include <cmath>
#include <cstddef>

namespace lidar_viewer {
// A bounded arithmetic mean of completed processing jobs, never rendered frames.
class ProcessingAverage {
public:
  static constexpr std::size_t Window=30;
  void Add(double milliseconds) {
    if(!std::isfinite(milliseconds) || milliseconds<0) return;
    if(count_==Window) total_-=values_[next_];else ++count_;
    values_[next_]=milliseconds;total_+=milliseconds;last_=milliseconds;next_=(next_+1)%Window;
  }
  void Reset() {*this={};}
  std::size_t Count() const {return count_;}
  double MeanMs() const {return count_ ? total_/static_cast<double>(count_) : 0.;}
  double LastMs() const {return last_;}
private:
  std::array<double,Window> values_{};
  std::size_t count_{0},next_{0};double total_{0},last_{0};
};
} // namespace lidar_viewer
