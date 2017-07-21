/**
 * @file ColorHistogram.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#ifndef Z_COLOR_HISTOGRAM_HH
#define Z_COLOR_HISTOGRAM_HH

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include "Color.hh"

namespace Z
{


/**
 * @brief Class ColorHistogram
 */
class ColorHistogram
{
private:
  int nr_points;
  int nr_bins;
  std::vector<cv::Vec4f> points;
  
  double *redHist, *greenHist, *blueHist;   /// r,g,b histogram
  double *yHist, *uHist, *vHist;            /// y,u,v histogram
  
public:
  ColorHistogram(int _nr_bins, std::vector<cv::Vec4f> _p);
  
  double Compare(ColorHistogram *ch);
  void PrintHistogram();
};

}

#endif

