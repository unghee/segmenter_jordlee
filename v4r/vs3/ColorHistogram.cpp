/**
 * @file ColorHistogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#include <string.h>
#include <cstdio>
#include "ColorHistogram.h"

namespace Z
{

ColorHistogram::ColorHistogram(int _nr_bins, std::vector<cv::Vec4f> _p)
{
  nr_bins = _nr_bins;
  points = _p;
  nr_points = points.size();

  redHist = new double[nr_bins];
  greenHist = new double[nr_bins];
  blueHist = new double [nr_bins];
  yHist = new double[nr_bins];
  uHist = new double[nr_bins];
  vHist = new double [nr_bins];
  
  for(unsigned i=0; i<nr_bins; i++)
  {
    redHist[i] = greenHist[i] = blueHist[i] = 0.;
    yHist[i] = uHist[i] = vHist[i] = 0.;
  }

  Z::RGBValue color;
  double bin = 0.;
  for(unsigned i=0; i<points.size(); i++)
  {
    color.float_value = points[i][3];
    
    bin = color.r*(double)nr_bins/255.;
    redHist[(int)bin]+=1;
    bin = color.g*(double)nr_bins/255.;
    greenHist[(int)bin]+=1;
    bin = color.b*(double)nr_bins/255.;
    blueHist[(int)bin]+=1;
  }
  for(unsigned i=0; i<nr_bins; i++)
  {
    redHist[i] /= nr_points;
    greenHist[i] /= nr_points;
    blueHist[i] /= nr_points;
  }
  
  // calculate YUV-histogram
  for(unsigned i=0; i<points.size(); i++)
  {
    color.float_value = points[i][3];
    int Y =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
    int U = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
    int V =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;

    bin = Y*(double)nr_bins/255.;
    yHist[(int)bin]+=1;
    bin = U*(double)nr_bins/255.;
    uHist[(int)bin]+=1;
    bin = V*(double)nr_bins/255.;
    vHist[(int)bin]+=1;
  }
  for(unsigned i=0; i<nr_bins; i++)
  {
    yHist[i] /= nr_points;
    uHist[i] /= nr_points;
    vHist[i] /= nr_points;
  }
}


double ColorHistogram::Compare(ColorHistogram *ch)
{
  if(nr_bins != ch->nr_bins)
  {
    std::printf("[ColorHistogram::Compare] Warning: Cannot compare histograms with different bin sizes.\n");
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
  
  
  /// RGB: Fidelity d=(SUM(sqrt(Pi*Qi)))
  /// RGB: Bhattacharyya d=-ln(SUM(sqrt(Pi*Qi)))
//   double overall_sum = 0;
//   for(unsigned i=0; i<nr_bins; i++)
//   {
//     overall_sum += sqrt(redHist[i]*ch->redHist[i]);
//     overall_sum += sqrt(greenHist[i]*ch->greenHist[i]);
//     overall_sum += sqrt(blueHist[i]*ch->blueHist[i]);
//   }  
//   overall_sum /= 3; // 3-color-channels
//   double fidelity = overall_sum;
//   double bhattacharyya = -log(overall_sum);
//   printf("overall_sum: %4.3f => %4.3f\n", overall_sum, bhattacharyya);
//   return fidelity;

  /// YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for(unsigned i=0; i<nr_bins; i++)
  {
//     overall_sum += sqrt(yHist[i]*ch->yHist[i]);
    overall_sum += sqrt(uHist[i]*ch->uHist[i]);
    overall_sum += sqrt(vHist[i]*ch->vHist[i]);
  }  
  overall_sum /= 2; // 3-color-channels
  double fidelity = overall_sum;
//   double bhattacharyya = -log(overall_sum);
//   printf("overall_sum: %4.3f => %4.3f\n", overall_sum, bhattacharyya);
  return fidelity;
}

void ColorHistogram::PrintHistogram()
{
  printf("Print histogram:\n");
  for(unsigned i=0; i<nr_bins; i++)
    printf(" red[%u]: %4.3f\n", i, redHist[i]);
  for(unsigned i=0; i<nr_bins; i++)
    printf(" gre[%u]: %4.3f\n", i, greenHist[i]);
  for(unsigned i=0; i<nr_bins; i++)
    printf(" blu[%u]: %4.3f\n", i, blueHist[i]);
}
  
}

