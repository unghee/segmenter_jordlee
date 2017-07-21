/**
 * @file ColorHistogram.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#ifndef SURFACE_COLOR_HISTOGRAM_HH
#define SURFACE_COLOR_HISTOGRAM_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include "v4r/PCLAddOns/PCLUtils.h"

namespace surface
{


/**
 * @brief Class ColorHistogram
 */
class ColorHistogram
{
private:
  int nr_bins;
  int color_model;                          /// 0 ... yuv / 1 ... rgb 
  
  bool computed;
  bool have_input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  double *redHist, *greenHist, *blueHist;   /// r,g,b histogram
  double *yHist, *uHist, *vHist;            /// y,u,v histogram
  
public:
  ColorHistogram(int _nr_bins);
  
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud);
  void setIndices(pcl::PointIndices::Ptr &_indices);
  void setColorModelYUV() {color_model = 0;}
  void setColorModelRGB() {color_model = 1;}
  void compute();
  double compare(ColorHistogram *ch);
  
  void printHistogram();
};

}

#endif

