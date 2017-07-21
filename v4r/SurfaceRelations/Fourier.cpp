/**
 * @file Fourier.cpp
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use fourier filter to compare surface texture.
 */

#include "Fourier.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Fourier::Fourier()
{
  N = 8;
  kmax = 5;
  nbins = 8;
  binWidth = 32;        /// TODO 8x binWidth is higher than max-range of values
  binStretch = 2.;      // optimal = 8
  
  have_input_image = false;
}

Fourier::~Fourier()
{
}

// ================================= Private functions ================================= //


// ================================= Public functions ================================= //


void Fourier::setInputImage(cv::Mat_<cv::Vec3b> &_rgb_image)
{
  cv::cvtColor( _rgb_image, gray_image, CV_RGB2GRAY); 
  have_input_image = true;
  computed = false;
}

void Fourier::compute()
{
  if(!have_input_image) {
    printf("[Fourier::compute] Error: No input image available. Abort.\n");
    return;
  }
  if( gray_image.empty()) {
    printf("[Fourier::compute] Error: Image is empty. Abort.\n");
    return;
  }
  double SX_r[kmax];
  double SX_i[kmax];
  double x[N];

  double en_r[kmax][N], en_i[kmax][N];       // real an imaginary part
  for(int k=0; k<kmax; k++) {
    for(int n=0; n<N; n++) {
      en_r[k][n] = cos(-2*M_PI/N * k*n);     // magnitude: x*e^(j*phi) = x*(cos(phi) + j*sin(phi))
      en_i[k][n] = sin(-2*M_PI/N * k*n);
    }
  }
   
  /// TODO calculation of the dft not on the borders of the image [1 ... n-1][1 ... m-1]
  for(int row = 1; row<gray_image.size().height-1; row++) {
    for(int col = 1; col<gray_image.size().width-1; col++) {
      x[0] = (double) gray_image.at<uchar>(row, col+1);
      x[1] = (double) gray_image.at<uchar>(row-1, col+1);
      x[2] = (double) gray_image.at<uchar>(row-1, col);
      x[3] = (double) gray_image.at<uchar>(row-1, col-1);
      x[4] = (double) gray_image.at<uchar>(row, col-1);
      x[5] = (double) gray_image.at<uchar>(row+1, col-1);
      x[6] = (double) gray_image.at<uchar>(row+1, col);
      x[7] = (double) gray_image.at<uchar>(row+1, col+1);
      for(int k=0; k<kmax; k++){
        SX_r[k] = 0.;
        SX_i[k] = 0.;
        for(int n=0; n<N; n++) {
          SX_r[k] += x[n]*en_r[k][n];
          SX_i[k] += x[n]*en_i[k][n];
        }
//         SX_r[k] /= (double) N;
//         SX_i[k] /= (double) N;
// printf("Fourier::compute: SX_r: %4.3f - SX_i: %4.3f\n", SX_r[k], SX_i[k]);
// printf("Fourier::compute: %u-dft: %3.3f \n", k, (SX_r[k]*SX_r[k] + SX_i[k]*SX_i[k]));
        double mag = sqrt(SX_r[k]*SX_r[k] + SX_i[k]*SX_i[k]) / ((double) N);
        
        int Xk = (int) mag; //(sqrt(SX_r[k]*SX_r[k] + SX_i[k]*SX_i[k]));
// printf("Fourier::compute: xk: %u\n", Xk);
        dft[col][row][k] = (uchar) Xk;
// printf("Fourier::compute: val - dft end: %u\n", dft[col][row][k]);
      }
    }
  }
  computed = true;
}
  
void Fourier::check()
{
  if(!computed) {
    printf("[Fourier::check] Error: DFT not computed. Abort.\n");
    return;
  }

  cv::Mat_<uchar> X_image[5];
  X_image[0] = cv::Mat_<uchar>(gray_image.size().height, gray_image.size().width);
  X_image[1] = cv::Mat_<uchar>(gray_image.size().height, gray_image.size().width);
  X_image[2] = cv::Mat_<uchar>(gray_image.size().height, gray_image.size().width);
  X_image[3] = cv::Mat_<uchar>(gray_image.size().height, gray_image.size().width);
  X_image[4] = cv::Mat_<uchar>(gray_image.size().height, gray_image.size().width);

  int bins[kmax][nbins];
  for(int k=0; k<kmax; k++)
    for(int n=0; n<N; n++)
      bins[k][n] = 0;
    
  for(int row = 1; row<gray_image.size().height-1; row++) {
    for(int col = 1; col<gray_image.size().width-1; col++) {
      for(int k=0; k<kmax; k++) {  
        X_image[k].at<uchar>(row, col) = dft[col][row][k];
        if(k == 0) { // 8 bins
          int bin = (int) (dft[col][row][k]/binWidth);
          bins[k][bin]++;
        }
        else {
          int bin = (int) (dft[col][row][k]/binWidth*binStretch);
          if(bin < nbins)
            bins[k][bin]++;
          else
            printf("Fourier: Warning: bin size too big.\n");
        } 
      }
    }
  }

  cv::imshow("X_image[0]", X_image[0]);
  cv::imshow("X_image[1]", X_image[1]);
  cv::imshow("X_image[2]", X_image[2]);
  cv::imshow("X_image[3]", X_image[3]);
  cv::imshow("X_image[4]", X_image[4]);
  
  for(int k=0; k<kmax; k++)
    for(int n=0; n<N; n++)
      printf("bin[%u][%u]: %u\n", k, n, bins[k][n]);
}
 

double Fourier::compare(std::vector<int> indices_0, std::vector<int> indices_1)
{
  if(!computed) {
    printf("[Fourier::compare] Error: DFT not computed. Abort.\n");
    return 0.;
  }
  
  double bins_0[kmax][nbins];
  double bins_1[kmax][nbins];
  for(int k=0; k<kmax; k++) {
    for(int n=0; n<N; n++) {
      bins_0[k][n] = 0.;
      bins_1[k][n] = 0.;
    }
  }

  for(std::size_t i=0; i< indices_0.size(); i++) {
    int row = indices_0[i] / gray_image.size().width;
    int col = indices_0[i] % gray_image.size().width;

    for(int k=0; k<kmax; k++) {  
      if(k == 0) { // 8 bins
        int bin = (int) (dft[col][row][k]/((double)binWidth));
        bins_0[k][bin]+=1;
      }
      else {
        int bin = (int) (dft[col][row][k]/((double) binWidth/(binStretch+(6*k))));
        if(bin < nbins)
          bins_0[k][bin]+=1;
        else {
          //printf("[Fourier::compare] Error: Left bin streching too much (k=%u) ", k);
          bin = nbins-1;
          bins_0[k][bin]+=1;
        }
      } 
    }
  }

  for(std::size_t i=0; i< indices_1.size(); i++) {
    int row = indices_1[i] / gray_image.size().width;
    int col = indices_1[i] % gray_image.size().width;

    for(int k=0; k<kmax; k++) {  
      if(k == 0) { // 8 bins
        int bin = (int) (dft[col][row][k]/((double)binWidth));
        bins_1[k][bin]+=1;
      }
      else {
        int bin = (int) (dft[col][row][k]/((double) binWidth/(binStretch+(6*k))));
        if(bin < nbins)
          bins_1[k][bin]+=1;
        else {
          //printf("[Fourier::compare] Error: Right bin streching too much (k=%u) ", k);
          bin = nbins-1;
          bins_1[k][bin]+=1;
        }
      } 
    }
  }

  
  // normalize the 40 values of each dft to sum=1
//   for(int k=0; k<kmax; k++)
//     for(int n=0; n<N; n++) {
//       bins_0[k][n] /= (indices_0.size()*kmax);
//       bins_1[k][n] /= (indices_1.size()*kmax);
//     }
// 
//   // Compare histogram with fidelity
//   double fidelity = 0.;
//   for(int k=0; k<kmax; k++) {
//     for(int n=0; n<N; n++) {
//       fidelity += sqrt(bins_0[k][n] * bins_1[k][n]);
//     }
//   }


  // TODO normalize from k=1
  for(int k=0; k<kmax; k++)
    for(int n=0; n<N; n++) {
      bins_0[k][n] /= (indices_0.size()*(kmax-1));
      bins_1[k][n] /= (indices_1.size()*(kmax-1));
    }

  double fidelity = 0.;
  for(int k=1; k<kmax; k++) {
    for(int n=0; n<N; n++) {
      fidelity += sqrt(bins_0[k][n] * bins_1[k][n]);
    }
  }  

//   // Compare histogram with euclidean distance
//   double euclidean = 0.;
//   for(int k=0; k<kmax; k++) {
//     for(int n=0; n<N; n++) {
//       euclidean += fabs(bins_0[k][n] - bins_1[k][n]);
//     }
//   }
//   euclidean = sqrt(euclidean);
//   
//   // print results
//   for(int k=0; k<kmax; k++)
//     for(int n=0; n<N; n++)
//       printf("bin[%u][%u]: %3.3f-%3.3f => %3.3f\n", k, n, bins_0[k][n], bins_1[k][n], sqrt(bins_0[k][n] * bins_1[k][n]));  
// 
//   printf("[Fourier::compare] fidelity: %4.3f / euclidean: %4.3f\n", fidelity, euclidean);
  
  return fidelity;
}

}


  /// OpenCV dft implementation
//   cv::Mat padded;                            //expand input image to optimal size
//   int m = cv::getOptimalDFTSize( gray_image.rows );
//   int n = cv::getOptimalDFTSize( gray_image.cols ); // on the border add zero values
//   cv::copyMakeBorder(gray_image, padded, 0, m - gray_image.rows, 0, n - gray_image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
// 
//   cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
//   cv::Mat complexI;
//   cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
// 
//   cv::dft(complexI, complexI);            // this way the result may fit in the source matrix
// 
//   // compute the magnitude and switch to logarithmic scale
//   // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
//   cv::split(complexI, planes);                      // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
//   cv::magnitude(planes[0], planes[1], planes[0]);   // planes[0] = magnitude  
//   cv::Mat magI = planes[0];
// 
// // cv::Mat show = planes[0];
// // cv::normalize(magI, show, 0, 1, CV_MINMAX); // Transform the matrix with float values into a 
// // cv::imshow("show", show);  
// 
//   magI += cv::Scalar::all(1);                    // switch to logarithmic scale
//   cv::log(magI, magI);
// 
//   // crop the spectrum, if it has an odd number of rows or columns
//   magI = magI(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));
// 
//   // rearrange the quadrants of Fourier image  so that the origin is at the image center        
//   int cx = magI.cols/2;
//   int cy = magI.rows/2;
// 
//   cv::Mat q0(magI, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant 
//   cv::Mat q1(magI, cv::Rect(cx, 0, cx, cy));  // Top-Right
//   cv::Mat q2(magI, cv::Rect(0, cy, cx, cy));  // Bottom-Left
//   cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy)); // Bottom-Right
// 
//   cv::Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
//   q0.copyTo(tmp);
//   q3.copyTo(q0);
//   tmp.copyTo(q3);
// 
//   q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
//   q2.copyTo(q1);
//   tmp.copyTo(q2);
// 
//   cv::normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a 
//                                           // viewable image form (float between values 0 and 1).
// 
//   cv::imshow("Fourier Input Image"       , gray_image   );    // Show the result
//   cv::imshow("Fourier Spectrum Magnitude", magI);  














