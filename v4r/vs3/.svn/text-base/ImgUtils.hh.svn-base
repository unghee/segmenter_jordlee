/**
 * $Id$
 */

#ifndef Z_IMG_UTILS_HH
#define Z_IMG_UTILS_HH

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <stdexcept>

namespace Z
{


/********************* set image pixel *****************************/
inline void SetPx8UC1(IplImage *img, short x, short y, uchar c)
{
  ((uchar*)(img->imageData + img->widthStep*y))[x] = c;
}

inline void SetPx16SC1(IplImage *img, short x, short y, short c)
{
  ((short*)(img->imageData + img->widthStep*y))[x] = c;
}

inline void SetPx8UC3(IplImage *img, short x, short y, uchar r, uchar g, uchar b)
{
  uchar *d =  &((uchar*)(img->imageData + img->widthStep*y))[x*3];
  d[0] = r;
  d[1] = g;
  d[2] = b;
}

inline void SetPx32FC1(IplImage *img, short x, short y, float c)
{
  ((float*)(img->imageData + img->widthStep*y))[x] = c;
}


/******************** get image pixel *******************************/
inline uchar GetPx8UC1(IplImage *img, short x, short y)
{
  return ((uchar*)(img->imageData + img->widthStep*y))[x];
}

inline uchar* GetPx8UC3(IplImage *img, short x, short y)
{
  return &((uchar*)(img->imageData + img->widthStep*y))[x*3];
}

inline float GetPx32FC1(IplImage *img, short x, short y)
{
  return ((float*)(img->imageData + img->widthStep*y))[x];
}

inline short GetPx16SC1(IplImage *img, short x, short y)
{
  return ((short*)(img->imageData + img->widthStep*y))[x];
}

/******************** test image format ****************************/
inline bool IsImage8UC1(IplImage *img)
{
  if (img->depth!=IPL_DEPTH_8U || img->nChannels!=1)
    return false;
  return true;
}

inline bool IsImage8UC3(IplImage *img)
{
  if (img->depth!=IPL_DEPTH_8U || img->nChannels!=3)
    return false;
  return true;
}

inline bool IsImage16SC1(IplImage *img)
{
  if (img->depth!=(int)IPL_DEPTH_16S || img->nChannels!=1)
    return false;
  return true;
}

inline bool IsImage32FC1(IplImage *img)
{
  if (img->depth!=(int)IPL_DEPTH_32F || img->nChannels!=1)
    return false;
  return true;
}

inline bool IsImage32FC3(IplImage *img)
{
  if (img->depth!=(int)IPL_DEPTH_32F || img->nChannels!=3)
    return false;
  return true;
}

inline bool IsImageSizeEqual(IplImage *img1, IplImage *img2)
{
  if (img1->width!=img2->width || img1->height!=img2->height)
    return false;
  return true;
}

inline bool IsImageEqual(IplImage *img1, IplImage *img2)
{
  if (img1->width!=img2->width || img1->height!=img2->height || 
      img1->depth!=img2->depth || img1->nChannels!=img2->nChannels)
    return false;
  return true;
}

}
  #endif
