/**
 * @file Gabor.cpp
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use garbor filter to compare surface texture.
 */

#include "Gabor.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Gabor::Gabor()
{
  deb = false;
  use_dilation = false;
  
  have_input_image = false;
  is_computed = false;

  F = sqrt(1.5);    // Spatial frequency (usually 2, but 1.5 works better)
  Sigma = 2*PI;     //
  N = 6;            // Number of orientations 0,30,60,90,120,150
  M_min = -2;       // Minimum scale factor (icpr=-2)
  M_max = 2;        // Maximum scale factor (icpr=2)
  size = N*abs(M_max - M_min +1);

  gabor = new CvGabor();
}

Gabor::~Gabor()
{
}

// ================================= Private functions ================================= //


// ================================= Public functions ================================= //

void Gabor::setDilation(unsigned _size)
{
  use_dilation = true;
  dilation_size = _size;
}

void Gabor::setInputImage(cv::Mat_<cv::Vec3b> &_rgb_image)
{
// printf("Gabor::setInputImage mat!\n");
  cv::Mat_<cv::Vec3b> rgb_image;
  cv::cvtColor( rgb_image, gray_image, CV_RGB2GRAY); 
  have_input_image = true;
  is_computed = false;
// printf("Gabor::setInputImage mat done!\n");
}

void Gabor::setInputImage(IplImage *_image)
{
// printf("Gabor::setInputImage!\n");
  ipl_gray_image = cvCreateImage(cvGetSize(_image), 8, 1);
  cvCvtColor(_image, ipl_gray_image, CV_RGB2GRAY);  
  have_input_image = true;
  is_computed = false;
// printf("Gabor::setInputImage done!\n");
}


void Gabor::compute()
{
  if(deb) printf("[Gabor::compute] start!\n");
  if(!is_computed)
  {
    unsigned res_image_cnt = 0;
    gabor_filter = new IplImage*[size];
    
    for(int ori=0; ori<N; ori++) {                    // orientation
      for(int scale=M_min; scale<=M_max; scale++) {   // scale
        double orientation = (PI*ori/N);  
        gabor_filter[res_image_cnt] = cvCreateImage(cvGetSize(ipl_gray_image), 8, 1);
        gabor->Init(orientation, scale, Sigma, F);
        if(deb) printf("[Gabor::compute] conv with orientation: %4.3f and scale: %i\n", orientation, scale);
        gabor->conv_img(ipl_gray_image, gabor_filter[res_image_cnt], CV_GABOR_MAG);
        res_image_cnt++;
      }
    }
    
    /// Draw results of gabor
//     cvShowImage("ipl_gray_image", ipl_gray_image);
//     cvShowImage("res_image_0", gabor_filter[0]);
//     cvShowImage("res_image_1", gabor_filter[5]);
//     cvShowImage("res_image_2", gabor_filter[10]);
//     cvShowImage("res_image_3", gabor_filter[15]);
//     cvShowImage("res_image_4", gabor_filter[20]);
//     cvShowImage("res_image_5", gabor_filter[25]);

    is_computed = true; 
  }
  if(deb) printf("[Gabor::compute] DONE!\n");
}


/// TODO TODO Eigentlich sollte man für alle Masken im Bild die gabor features
/// ausrechnen und dann speichern für die Abfragen? Das Selbe gilt auch für die 
/// Fourier-Berechnung (wird immer wieder für die Selbe maske gemacht)
double Gabor::compare(std::vector<int> mask_0, std::vector<int> mask_1)
{
  bool print = false;
  
  bool normalisaton_by_energy = true;
  double normalise = 1.0f;    // normalise = 255                                        /// TODO Untested
  
  // calculate mean and standard-deviaton of the magnitude for each 
  // gabor-filter (5x6) for each mask and create the feature vector
  // => Magnitude = Energy content
  // Then calculate the distance between the feature vectors.
 
  // Dilation of masks
  if(use_dilation) {
    printf("[Gabor::compare] Start of dilation: %u\n", dilation_size);
    MaskDilationErosion erosion;
    erosion.setImageSize(ipl_gray_image->width, ipl_gray_image->height);
    erosion.setSize(dilation_size);
    erosion.compute(mask_0);
    erosion.compute(mask_1);
    printf("[Gabor::compare] End of erosion!\n");
  }
 
  /// Calculate mean and standard deviation for each mask.
  int vec_size = size*2;
  double feature_vector_0[vec_size];  // scale * orientation *2 (mean/st.devi)
  double feature_vector_1[vec_size];  // scale * orientation *2
  double new_feature_vector_0[vec_size];    // after shifting  
  double new_feature_vector_1[vec_size];
  double mean_mask_0 = 0.0f;
  double mean_mask_1 = 0.0f;
  double mu_mask_0 = 0.0f;
  double mu_mask_1 = 0.0f;
 
  /// calculate mean value
  for(unsigned idx=0; idx<size; idx++) {
    for(unsigned i=0; i<mask_0.size(); i++)
      mean_mask_0 += ((double) gabor_filter[idx]->imageData[mask_0[i]] / normalise);
    mean_mask_0 /= (double) mask_0.size();
    for(unsigned i=0; i<mask_1.size(); i++) {
      mean_mask_1 += ((double) gabor_filter[idx]->imageData[mask_1[i]] / normalise);
    }
    mean_mask_1 /= (double) mask_1.size();
    feature_vector_0[idx*2] = mean_mask_0;
    feature_vector_1[idx*2] = mean_mask_1;
  }

  /// calculate standard deviation
  for(unsigned idx=0; idx<size; idx++) // filters
  {
    for(unsigned i=0; i<mask_0.size(); i++)
      mu_mask_0 += pow(((double) gabor_filter[idx]->imageData[mask_0[i]] / normalise) - mean_mask_0, 2);
    mu_mask_0 /= (double) (mask_0.size()-1);
    mu_mask_0 = sqrt(mu_mask_0);
    
    for(unsigned i=0; i<mask_1.size(); i++)
      mu_mask_1 += pow(((double) gabor_filter[idx]->imageData[mask_1[i]] / normalise) - mean_mask_1, 2);
    mu_mask_1 /= (double) (mask_1.size()-1);
    mu_mask_1 = sqrt(mu_mask_1);

    feature_vector_0[idx*2 +1] = mu_mask_0;
    feature_vector_1[idx*2 +1] = mu_mask_1;
  }

  
/// print feature vectors
// printf("vector_0: ");
// for(unsigned i=0; i<size*2; i++) {
//   printf("%2.1f - ", feature_vector_0[i]);
// }
// printf("\nvector_1: ");
// for(unsigned i=0; i<size*2; i++) {
//   printf("%2.1f - ", feature_vector_1[i]);
// }
// printf("\n");
  
  
  /// Now find the orientation with highest energy
  double ori_mag_sum_0[N], ori_mag_sum_1[N];        // sum of magnitude (energy) for one orientation
  for(int i=0; i<N; i++) {
    ori_mag_sum_0[i] = 0.0f;
    ori_mag_sum_1[i] = 0.0f;
  }
  
  for(int ori=0; ori<N; ori++)                      // orientations
    for(unsigned i=0; i<(unsigned) size/N; i++) {   // scales
      int pos = (ori*size/N+i)*2;
// printf("sum up ori[%u] from pos: %u\n", ori, pos);
      ori_mag_sum_0[ori] += feature_vector_0[pos];
      ori_mag_sum_1[ori] += feature_vector_1[pos];
    }
    
// for(int ori=0; ori<N; ori++)
//   printf("ori_mag_sum_x[%u]: %4.3f - %4.3f\n", ori, ori_mag_sum_0[ori], ori_mag_sum_1[ori]);
    
  unsigned max_ori_nr_0 = 0;
  unsigned max_ori_nr_1 = 0;
  double max_ori_0 = 0.0f;
  double max_ori_1 = 0.0f;
  for(int i=0; i<N; i++) {
    if(ori_mag_sum_0[i] > max_ori_0) {
      max_ori_0 = ori_mag_sum_0[i];
      max_ori_nr_0 = i;
    }
    if(ori_mag_sum_1[i] > max_ori_1) {
      max_ori_1 = ori_mag_sum_1[i];
      max_ori_nr_1 = i;
    }
// printf("ori sum [%u]: %4.3f - %4.3f\n", i, max_ori_0, max_ori_1);
  }
  
if(print) printf("[Gabor::compare] max_ori_x: %4.3f - %4.3f\n", max_ori_0, max_ori_1);
if(print) printf("[Gabor::compare] highest oris: %u-%u\n", max_ori_nr_0, max_ori_nr_1);

  /// normalisaton value is the maximum energy in one direction (orientation)
  double norm_energy = fmax(max_ori_0, max_ori_1);

  // Shift the feature vector to get the highest output energy at first orientation row
  /// Shift, when orientation is different
  if(max_ori_nr_0 != max_ori_nr_1)
  {
    unsigned shift = 2*max_ori_nr_0*size/N;
    for(unsigned pos=0; pos<size*2; pos++) {
      unsigned newpos = (pos+shift) % (size*2);
      new_feature_vector_0[pos] = feature_vector_0[newpos];
// printf("  %u => %u: %2.1f => %2.1f\n", newpos, pos, feature_vector_0[pos], new_feature_vector_0[newpos]);
    }
    shift = 2*max_ori_nr_1*size/N;
    for(unsigned pos=0; pos<size*2; pos++) {
      unsigned newpos = (pos+shift) % (size*2);
      new_feature_vector_1[pos] = feature_vector_1[newpos];
// printf("  %u => %u: %2.1f => %2.1f\n", newpos, pos, feature_vector_1[pos], new_feature_vector_1[newpos]);
    }
    for(unsigned pos=0; pos<size*2; pos++) {
      feature_vector_0[pos] = new_feature_vector_0[pos];
      feature_vector_1[pos] = new_feature_vector_1[pos];
    }
  }
  
/// print shifted feature vectors
// printf("s_vector_0: ");
// for(unsigned i=0; i<size*2; i++) {
//   printf("%2.1f - ", feature_vector_0[i]);
// }
// printf("\ns_vector_1: ");
// for(unsigned i=0; i<size*2; i++) {
//   printf("%2.1f - ", feature_vector_1[i]);
// }
// printf("\n");

  
  /// NOW calculate the distance of the feature vectors
  /// sqrt((mu_n-mu_m)² + (si_n-si_m)²)
  double gabor_distance = 0.0f;
  for(unsigned i=0; i<size*2; i+=2) {
    double mu = pow(feature_vector_0[i] - feature_vector_1[i], 2);
    double sig = pow(feature_vector_0[i+1] - feature_vector_1[i+1], 2);
    gabor_distance += sqrt(mu + sig);
  }
  
if(print) printf("[Gabor::compare] Gabor normalise distance: %4.3f\n", gabor_distance/norm_energy);
  
  if(gabor_distance == gabor_distance) {  // nan check
    if(normalisaton_by_energy) 
    {
      if(norm_energy != 0.)
        return gabor_distance/norm_energy;
      else 
        return 10.0;          // TODO Return value?
    }
    else
      return gabor_distance;
  }
  else 
    printf("[Gabor::compare] Error: Gabor distance is nan: %4.3f\n", gabor_distance);
  return 10.0;                // TODO Return value?
}

}
