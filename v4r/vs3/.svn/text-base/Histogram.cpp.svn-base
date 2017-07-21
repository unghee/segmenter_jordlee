/**
 * @file Histogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Histogram class.
 */

#include <string.h>
#include "v4r/vs3/Math.hh"
#include "Histogram.h"

namespace Z
{

Histogram::Histogram(int _nr_bins, std::vector<double> &_p)
{
  nr_bins = _nr_bins;
  points = _p;

  histogram = new double[nr_bins];
  
  for(unsigned i=0; i<nr_bins; i++)
    histogram[i] = 0.;

  for(unsigned i=0; i<points.size(); i++)
  {
    double bin = points[i]*nr_bins;
    histogram[(int)bin] += 1;
  }
   
  for(unsigned i=0; i<nr_bins; i++)
    histogram[i] /= points.size();
  
  PrintHistogram();  /// TODO Print histogram
}


double Histogram::Compare(Histogram *h)
{
  double this_sum = 0.;
  double h_sum = 0.;
  
  if(nr_bins != h->nr_bins)
  {
    printf("Histogram::Compare: Warning: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  
  for(unsigned i=0; i<nr_bins; i++)               // TODO TODO TODO TODO TODO TODO TODO TODO How to compare? Gewichtet?
  {
    this_sum += histogram[i]*i/nr_bins;
    h_sum += h->histogram[i]*i/nr_bins;
  }
  return fabs(this_sum - h_sum);
}


void Histogram::PrintHistogram()
{
  printf("Printed histogram:\n");
  for(unsigned i=0; i<nr_bins; i++)
    printf(" histogram[%u]: %4.3f\n", i, histogram[i]);
}
  
}

