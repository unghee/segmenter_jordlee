/**
 * @file Fourier.h
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use fourier filter to compare surface texture.
 */

#ifndef SURFACE_FOURIER_H
#define SURFACE_FOURIER_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <omp.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace surface
{

class Fourier
{
public:
  
protected:

private:

  int N;            // Number of neighbors
  int kmax;         // maximum number of discrete fourier transformation coefficient
  int nbins;        // number of histogram bins
  int binWidth;     // Width of one bin (32 width => 8x32 = 256)
  int binStretch;   // Stretch factor for bins of higher order (k=1,...)
  
  bool have_input_image;
  cv::Mat_<uchar> gray_image;           // gray level image (as float)

  bool computed;                        // dft is computed
  uchar dft[640][480][5];               // dft results for kmax = 5 coefficients
  
public:
  Fourier();
  ~Fourier();
  
  /** Set input image **/
  void setInputImage(cv::Mat_<cv::Vec3b> &_image);
//   void setInputImage(IplImage *_image);

  /** Compute the surf features **/
  void compute();
  
  /** Check the results visually on images for each dft-component **/
  void check();
  
  /** Compare surfaces **/
  double compare(std::vector<int> indices_0, std::vector<int> indices_1);
  
  /** Get reults from comparison **/
  


};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

