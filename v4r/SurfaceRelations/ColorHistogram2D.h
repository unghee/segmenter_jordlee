/**
 * @file ColorHistogram2D.h
 * @author Andreas Richtsfeld
 * @date February 2012
 * @version 0.1
 * @brief Color histogram class for 2D comparison.
 */

#ifndef SURFACE_COLOR_HISTOGRAM2D_HH
#define SURFACE_COLOR_HISTOGRAM2D_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include "v4r/PCLAddOns/PCLUtils.h"

namespace surface
{


/**
 * @brief Class ColorHistogram2D
 */
class ColorHistogram2D
{
private:
  int nr_bins;
  int color_model;                            /// 0 ... yuv / 1 ... rgb 
  double UVthreshold;

  bool computed;
  bool have_input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  std::vector< std::vector<double> > uvHist;  /// 2D u,v histogram
  
public:
  ColorHistogram2D(int _nr_bins);
  
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud);
  void setIndices(pcl::PointIndices::Ptr &_indices);
  void setColorModelYUV() {color_model = 0;}
  void setColorModelRGB() {color_model = 1;}
  void compute();
  double compare(ColorHistogram2D *ch);
  
  void printHistogram();
};

}

#endif

