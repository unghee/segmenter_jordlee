/**
 * @file Histogram.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Histogram class.
 */

#ifndef Z_HISTOGRAM_HH
#define Z_HISTOGRAM_HH

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include "Color.hh"

namespace Z
{


/**
 * @brief Class Histogram
 */
class Histogram
{
private:
  int nr_bins;                      // Nr of bins
  std::vector<double> points;       // Data points (normalised to 1)
  
  double *histogram;                // resulting histogram
  
public:
  Histogram(int _nr_bins, std::vector<double> &_p);
  ~Histogram() {delete histogram;}
  
  double* GetHistogram() {return histogram;}
  double Compare(Histogram *h);
  void PrintHistogram();
};

}

#endif

