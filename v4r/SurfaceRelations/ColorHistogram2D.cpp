/**
 * @file ColorHistogram2D.cpp
 * @author Andreas Richtsfeld
 * @date February 2012
 * @version 0.1
 * @brief Color histogram class for 2D comparison.
 */

#include <string.h>
#include "ColorHistogram2D.h"

namespace surface
{

ColorHistogram2D::ColorHistogram2D(int _nr_bins)
{
  nr_bins = _nr_bins;
  computed = false;
  have_input_cloud = false;
  have_indices = false;
  color_model = 0;
  UVthreshold = 10.;
}

void ColorHistogram2D::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud)
{
  input_cloud = _input_cloud;
  have_input_cloud = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(unsigned i=0; i<input_cloud->points.size(); i++)
    indices->indices.push_back(i);
}

void ColorHistogram2D::setIndices(pcl::PointIndices::Ptr &_indices)
{
  if(!have_input_cloud) {
    printf("ColorHistogram::setIndices: Error: No input cloud available.\n");
    return;
  }
  indices = _indices;
  have_indices = true;
}

void ColorHistogram2D::compute()
{
  uvHist.clear();
  for(int i=0; i< nr_bins; i++) {
    std::vector<double> hist;
    for(int j=0; j< nr_bins; j++)
      hist.push_back(0.0f);
    uvHist.push_back(hist);
  }
  
  if(color_model == 0)   // yuv
  {
    pclA::RGBValue color;
    int noCol = 0;
    double uBin = 0.;
    double vBin = 0.;
    for(unsigned i=0; i<indices->indices.size(); i++) {
      color.float_value = input_cloud->points[indices->indices[i]].rgb;
// //       int Y =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
      int U = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
      int V =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
//       int Y =  (0.257 * color.b) + (0.504 * color.g) + (0.098 * color.r) + 16;   /// TODO rgb or bgr?
//       int U = -(0.148 * color.b) - (0.291 * color.g) + (0.439 * color.r) + 128;
//       int V =  (0.439 * color.b) - (0.368 * color.g) - (0.071 * color.r) + 128;

      int U2 = U-128;        /// shifted to the middle
      int V2 = V-128;        /// shifted to the middle
      if((U2*U2 + V2*V2) < UVthreshold) 
        noCol++;
      else {
        uBin = U*(double)nr_bins/255.;
        vBin = V*(double)nr_bins/255.;
        uvHist[(int)uBin][(int)vBin] += 1;
      }
    }
    
    double normalization = indices->indices.size() - noCol;
    for(int i=0; i<nr_bins; i++)
      for(int j=0; j<nr_bins; j++)
        uvHist[i][j] /= normalization;
  }
  else if(color_model == 1) // rgb
  {
    printf("[ColorHistogram2D::compute] Warning: rgb model not supported at the moment!\n");
//     pclA::RGBValue color;
//     double bin = 0.;
//     for(unsigned i=0; i<indices->indices.size(); i++) {
//       color.float_value = input_cloud->points[indices->indices[i]].rgb;
//       bin = color.r*(double)nr_bins/255.;
//       redHist[(int)bin]+=1;
//       bin = color.g*(double)nr_bins/255.;
//       greenHist[(int)bin]+=1;
//       bin = color.b*(double)nr_bins/255.;
//       blueHist[(int)bin]+=1;
//     }
//     for(int i=0; i<nr_bins; i++) {
//       redHist[i] /= indices->indices.size();
//       greenHist[i] /= indices->indices.size();
//       blueHist[i] /= indices->indices.size();
//     }
  }
  computed = true;
}


double ColorHistogram2D::compare(ColorHistogram2D *ch)
{
  if(nr_bins != ch->nr_bins) {
    printf("[ColorHistogram2D::Compare] Error: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  if(!computed || !ch->computed) {
    printf("[ColorHistogram2D::Compare] Error: Color histogram not computed.\n");
    return 0.;
  }
  
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  if(color_model == 0) {
    double overall_sum = 0;
    for(int i=0; i<nr_bins; i++)
      for(int j=0; j<nr_bins; j++)
        overall_sum += sqrt(uvHist[i][j]*ch->uvHist[i][j]);
//     overall_sum /= 2; // 2 color-channels (U-V)
        
if(overall_sum > 1.) {
  printf("Warning: ColorHistogram2D::compare: Value larger than 1!!!\n");
  printHistogram();
}
        
    double fidelity = overall_sum;
  //   double bhattacharyya = -log(overall_sum);
  //   printf("overall_sum: %4.3f => %4.3f\n", overall_sum, bhattacharyya);
    return fidelity;
  }

  return 0.0;
}

void ColorHistogram2D::printHistogram()
{
  printf("Print histogram:\n");

  if(color_model == 0) {
    for(int i=0; i<nr_bins; i++)
      for(int j=0; j<nr_bins; j++) {
        printf(" [%u] [%u] : %4.3f\n", i, j, uvHist[i][j]);
//     for(int i=0; i<nr_bins; i++)
//       printf(" v[%u]: %4.3f\n", i, vHist[i]);
      }
  }
}
  
}

