/**
 * @file Gabor.h
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use garbor filter to compare surface texture.
 */

#ifndef SURFACE_GABOR_H
#define SURFACE_GABOR_H

#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cvgabor.h"
#include "MaskDilationErosion.h"


namespace surface
{

class Gabor
{
public:
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  bool deb;                             // debug output
  
  bool use_dilation;                    // set true to use dilation of mask
  unsigned dilation_size;               // size of dilation

  bool have_input_image;
  cv::Mat_<uchar> gray_image;           // gray level image in cv-Mat
  IplImage *ipl_gray_image;             // gray level image

  bool is_computed;                     // true, when results available
  
  double F;                             // dF ... The spatial frequency
  double Sigma;                         // TODO Sigma
  int N;                                // Number of orientations
  int M_min, M_max;                     // minimum and maximum scale factor
  unsigned size;                        // size of gabor filters (orientations * number scales)
  CvGabor *gabor;
  
  IplImage **gabor_filter;              // Gabor filter image saved as IplImage

public:
  Gabor();
  ~Gabor();
  
  /** Activate dilation and set size **/
  void setDilation(unsigned _size);
  
  /** Set input image **/
  void setInputImage(cv::Mat_<cv::Vec3b> &_image);
  void setInputImage(IplImage *_image);

  /** Compute the gabor features **/
  void compute();
  
  /** Compare surfaces with gabor filter **/
  double compare(std::vector<int> mask_0, std::vector<int> mask_1);
  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

