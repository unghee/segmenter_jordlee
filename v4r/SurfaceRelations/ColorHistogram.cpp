/**
 * @file ColorHistogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#include <string.h>
#include "ColorHistogram.h"

namespace surface
{

ColorHistogram::ColorHistogram(int _nr_bins)
{
  nr_bins = _nr_bins;
  computed = false;
  have_input_cloud = false;
  have_indices = false;
  color_model = 0;
}

void ColorHistogram::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_input_cloud)
{
  input_cloud = _input_cloud;
  have_input_cloud = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(unsigned i=0; i<input_cloud->points.size(); i++)
    indices->indices.push_back(i);
}

void ColorHistogram::setIndices(pcl::PointIndices::Ptr &_indices)
{
  if(!have_input_cloud) {
    printf("ColorHistogram::setIndices: Error: No input cloud available.\n");
    return;
  }
    
  indices = _indices;
  have_indices = true;
}

void ColorHistogram::compute()
{
  redHist = new double[nr_bins];
  greenHist = new double[nr_bins];
  blueHist = new double [nr_bins];
  yHist = new double[nr_bins];
  uHist = new double[nr_bins];
  vHist = new double [nr_bins];
  
  for(int i=0; i<nr_bins; i++)  {
    redHist[i] = greenHist[i] = blueHist[i] = 0.;
    yHist[i] = uHist[i] = vHist[i] = 0.;
  }

  if(color_model == 0)   // yuv
  {
    pclA::RGBValue color;
    double bin = 0.;
    for(unsigned i=0; i<indices->indices.size(); i++) {
      color.float_value = input_cloud->points[indices->indices[i]].rgb;
// //       int Y =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
//       int U = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
//       int V =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
//       int Y =  (0.257 * color.b) + (0.504 * color.g) + (0.098 * color.r) + 16;   /// TODO rgb or bgr?
      int U = -(0.148 * color.b) - (0.291 * color.g) + (0.439 * color.r) + 128;
      int V =  (0.439 * color.b) - (0.368 * color.g) - (0.071 * color.r) + 128;

//       bin = Y*(double)nr_bins/255.;
//       yHist[(int)bin]+=1;
      bin = U*(double)nr_bins/255.;
      uHist[(int)bin]+=1;
      bin = V*(double)nr_bins/255.;
      vHist[(int)bin]+=1;
    }
    for(int i=0; i<nr_bins; i++) {
      yHist[i] /= indices->indices.size();
      uHist[i] /= indices->indices.size();
      vHist[i] /= indices->indices.size();
    }
  }
  else if(color_model == 1) // rgb
  {
    pclA::RGBValue color;
    double bin = 0.;
    for(unsigned i=0; i<indices->indices.size(); i++) {
      color.float_value = input_cloud->points[indices->indices[i]].rgb;
      bin = color.r*(double)nr_bins/255.;
      redHist[(int)bin]+=1;
      bin = color.g*(double)nr_bins/255.;
      greenHist[(int)bin]+=1;
      bin = color.b*(double)nr_bins/255.;
      blueHist[(int)bin]+=1;
    }
    for(int i=0; i<nr_bins; i++) {
      redHist[i] /= indices->indices.size();
      greenHist[i] /= indices->indices.size();
      blueHist[i] /= indices->indices.size();
    }
  }
  computed = true;
}


double ColorHistogram::compare(ColorHistogram *ch)
{
  if(nr_bins != ch->nr_bins) {
    printf("ColorHistogram::Compare: Error: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  if(!computed || !ch->computed) {
    printf("ColorHistogram::Compare: Error: Color histogram not computed.\n");
    return 0.;
  }
  
  /// RGB: Compare with Aris weighted compare method
//   double overall_sum = 0;
//   double this_sum[3] = {0., 0., 0.};
//   double ch_sum[3] = {0., 0., 0.};
//   for(unsigned i=0; i<nr_bins; i++)
//   {
//     this_sum[0] += redHist[i]*i*1/nr_bins;
//     ch_sum[0] += ch->redHist[i]*i*1/nr_bins;
//     this_sum[1] += greenHist[i]*i*1/nr_bins;
//     ch_sum[1] += ch->greenHist[i]*i*1/nr_bins;
//     this_sum[2] += blueHist[i]*i*1/nr_bins;
//     ch_sum[2] += ch->blueHist[i]*i*1/nr_bins;
//   }
//   for(unsigned col=0; col<3; col++)
//     overall_sum += fabs(this_sum[col] - ch_sum[col]);
//   overall_sum /= 3.;
//   return overall_sum;
  
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  if(color_model == 0) {
    double overall_sum = 0;
    for(int i=0; i<nr_bins; i++)
    {
//       overall_sum += sqrt(yHist[i]*ch->yHist[i]);
      overall_sum += sqrt(uHist[i]*ch->uHist[i]);
      overall_sum += sqrt(vHist[i]*ch->vHist[i]);
    }  
    overall_sum /= 2; // 2 color-channels (U-V)
    double fidelity = overall_sum;
  //   double bhattacharyya = -log(overall_sum);
  //   printf("overall_sum: %4.3f => %4.3f\n", overall_sum, bhattacharyya);
    return fidelity;
  }
  
  // RGB: Fidelity d=(SUM(sqrt(Pi*Qi)))
  // RGB: Bhattacharyya d=-ln(SUM(sqrt(Pi*Qi)))
  else if(color_model == 1) {
    double overall_sum = 0;
    for(int i=0; i<nr_bins; i++)
    {
      overall_sum += sqrt(redHist[i]*ch->redHist[i]);
      overall_sum += sqrt(greenHist[i]*ch->greenHist[i]);
      overall_sum += sqrt(blueHist[i]*ch->blueHist[i]);
    }  
    overall_sum /= 3; // 3-color-channels (R-G-B)
    double fidelity = overall_sum;
//     double bhattacharyya = -log(overall_sum);
    // printf("overall_sum: %4.3f => %4.3f\n", overall_sum, bhattacharyya);
    return fidelity;
  }
  return 0.0;
}

void ColorHistogram::printHistogram()
{
  printf("Print histogram:\n");
  if(color_model == 1) {
    for(int i=0; i<nr_bins; i++)
      printf(" red[%u]: %4.3f\n", i, redHist[i]);
    for(int i=0; i<nr_bins; i++)
      printf(" gre[%u]: %4.3f\n", i, greenHist[i]);
    for(int i=0; i<nr_bins; i++)
      printf(" blu[%u]: %4.3f\n", i, blueHist[i]);
  }
  else if(color_model == 0) {
    for(int i=0; i<nr_bins; i++)
      printf(" y[%u]: %4.3f\n", i, yHist[i]);
    for(int i=0; i<nr_bins; i++)
      printf(" u[%u]: %4.3f\n", i, uHist[i]);
    for(int i=0; i<nr_bins; i++)
      printf(" v[%u]: %4.3f\n", i, vHist[i]);
  }
}
  
}

