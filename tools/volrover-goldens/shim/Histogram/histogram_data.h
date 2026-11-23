// The piece of legacy Histogram that legacy SecondaryStructures uses: HistogramData, the widths its
// GUI's histograms showed (Skel::get_alpha_histogram_data and get_beta_histogram_data). This one
// keeps the widths it was given, so secondary_driver.cpp can write them. Part of
// tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_HISTOGRAM_DATA_H
#define VOLROVER_GOLDENS_SHIM_HISTOGRAM_DATA_H

#include <vector>

class HistogramData {
public:
  HistogramData() = default;
  HistogramData(const std::vector<double> &inputPoints, unsigned int numBins)
      : points(inputPoints), bins(numBins) {}
  std::vector<double> points;
  unsigned int bins = 0;
};

#endif
