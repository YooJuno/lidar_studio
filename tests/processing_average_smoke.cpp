#include "lidar_viewer/processing_average.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

int main()
{
  using lidar_viewer::ProcessingAverage;
  const auto check=[](bool value) {if(!value) throw std::runtime_error("Invalid processing average");};
  ProcessingAverage average;
  check(average.Count()==0 && average.MeanMs()==0);
  average.Add(5);average.Add(15);
  check(average.Count()==2 && average.MeanMs()==10 && average.LastMs()==15);
  average.Add(-1);average.Add(std::numeric_limits<double>::quiet_NaN());
  average.Add(std::numeric_limits<double>::infinity());
  check(average.Count()==2 && average.MeanMs()==10);
  average.Reset();
  for(int i=1;i<=100;++i) average.Add(i);
  check(average.Count()==ProcessingAverage::Window && std::abs(average.MeanMs()-85.5)<1e-9);
  const auto snapshot=average;
  average.Reset();average.Add(0);
  check(average.Count()==1 && average.MeanMs()==0 && snapshot.LastMs()==100);
}
