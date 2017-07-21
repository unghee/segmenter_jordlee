/**
 * $Id$
 * 
 * Johann Prankl, 20091111
 *
 *  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 *
 *  By downloading, copying, installing or using the software you agree to this license.
 *  If you do not agree to this license, do not download, install,
 *  copy or use the software.
 *
 *
 *                        Intel License Agreement
 *                For Open Source Computer Vision Library
 *
 * Copyright (C) 2000, Intel Corporation, all rights reserved.
 * Third party copyrights are property of their respective owners.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   * Redistribution's of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   * Redistribution's in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   * The name of Intel Corporation may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * This software is provided by the copyright holders and contributors "as is" and
 * any express or implied warranties, including, but not limited to, the implied
 * warranties of merchantability and fitness for a particular purpose are disclaimed.
 * In no event shall the Intel Corporation or contributors be liable for any direct,
 * indirect, incidental, special, exemplary, or consequential damages
 * (including, but not limited to, procurement of substitute goods or services;
 * loss of use, data, or profits; or business interruption) however caused
 * and on any theory of liability, whether in contract, strict liability,
 * or tort (including negligence or otherwise) arising in any way out of
 * the use of this software, even if advised of the possibility of such damage.
 **/


#include "CEdge.hh"


namespace Z 
{


/********************** CEdge ************************
 * Constructor/Destructor
 */
CEdge::CEdge()
 : useCol(false), apertureSize(3)
{
}

CEdge::~CEdge()
{
}


void CEdge::SobelGrey(IplImage *img, IplImage *dx, IplImage *dy)
{
  if(!IsImage8UC1(img))
    throw std::runtime_error("CEdge::SobelGrey: Input image should be IPL_DEPTH_8U, 1 channels!");
  if(!IsImage16SC1(dx) || !IsImage16SC1(dy))
    throw std::runtime_error("CEdge::SobelGrey: Output images should be IPL_DEPTH_16S, 1 channel!");
  if(!IsImageSizeEqual(img,dx) || !IsImageSizeEqual(img,dy))
    throw std::runtime_error("CEdge::SobelGrey: Size of images must be equal");

  cvSobel( img, dx, 1, 0, apertureSize );
  cvSobel( img, dy, 0, 1, apertureSize );
}

void CEdge::SobelCol(IplImage *img, IplImage *dx, IplImage *dy)
{
  if(!IsImage8UC3(img))
    throw std::runtime_error("CEdge::SobelCol: Input image should be IPL_DEPTH_8U, 3 channels!");
  if(!IsImage16SC1(dx) || !IsImage16SC1(dy))
    throw std::runtime_error("CEdge::SobelCol: Output images should be IPL_DEPTH_16S, 1 channel!");
  if(!IsImageSizeEqual(img,dx) || !IsImageSizeEqual(img,dy))
    throw std::runtime_error("CEdge::SobelCol: Size of images must be equal");

  IplImage *edge = cvCreateImage(cvGetSize(img), IPL_DEPTH_16S, img->nChannels );
  short *d, *d_end, *e;

  cvSobel( img, edge, 1, 0, apertureSize );
  
  for (int v=0; v<edge->height; v++)
  {
    e = (short*)(edge->imageData + edge->widthStep*v);
    d = (short*)(dx->imageData + dx->widthStep*v);
    d_end = d + dx->width;

    for (;d!=d_end; d++, e+=3)
      *d = (short)(Max(e[0],e[1],e[2]));
      //*d = (short)(sqrt(Sqr((float)e[0]) + Sqr((float)e[1]) + Sqr((float)e[2])));
  }

  cvSobel( img, edge, 0, 1, apertureSize );

  for (int v=0; v<edge->height; v++)
  {
    e = (short*)(edge->imageData + edge->widthStep*v);
    d = (short*)(dy->imageData + dy->widthStep*v);
    d_end = d + dy->width;

    for (;d!=d_end; d++, e+=3)
    {
      *d = (short)(Max(e[0],e[1],e[2]));
      //*d = (short)(sqrt(Sqr((float)e[0]) + Sqr((float)e[1]) + Sqr((float)e[2])));
    }
  }

  cvReleaseImage(&edge);
}

/**
 * Trace edge gradient
 */
void CEdge::TraceEdge2(IplImage *edgels, IplImage *dx, IplImage *dy, IplImage *tmp, int u, int v, Array<Edgel> &arr)
{
  int x=u, y=v;
  Vector2 e;

  do
  {
    SetPx8UC1(tmp,x, y,255);
    e = Vector2(x,y);
    arr.PushBack(Edgel(e,atan2(y,x)));

  }while(TestEdgel(edgels, tmp, x,y) && y>0 && y<edgels->height-1);
}















/*********************** PUBLIC ******************************************/

/**
 * Sobel operator
 * ATTENTION:
 * If img is has 3 channels then the colour version is applied
 */
void CEdge::Sobel(IplImage *img, IplImage *dx, IplImage *dy)
{
  if (useCol)
    SobelCol(img, dx, dy);
  else
    SobelGrey(img, dx, dy);
}

/**
 * Get the norm of edgels according to dx and dy
 */
void CEdge::Norm(IplImage *dx, IplImage *dy, IplImage *edge)
{
  if(!IsImage16SC1(dx) || !IsImage16SC1(dy))
    throw std::runtime_error("CEdge::Norm: Input images should be IPL_DEPTH_16S, 1 channel!");
  if (!IsImageSizeEqual(dx,dy) || !IsImageSizeEqual(dx,edge))
    throw std::runtime_error("CEdge::Norm: Size of image must be equal!");
  if (edge->nChannels != 1)
    throw std::runtime_error("CEdge::Norm: Output image must be a 1 channel image!");

  short *d1, *d2;

  if (edge->depth == (int)IPL_DEPTH_16S)
  {
    short *e, *e_end; 

    for (int v=0; v<edge->height; v++)
    {
      d1 = (short*)(dx->imageData + dx->widthStep*v);
      d2 = (short*)(dy->imageData + dy->widthStep*v);
      e = (short*)(edge->imageData + edge->widthStep*v);
      e_end = e + edge->width;
      
      for (;e!=e_end; d1++,d2++,e++)
        *e = (short)(sqrt(Sqr((float)*d1) + Sqr((float)*d2)));
    }
  }
  else if (edge->depth == (int)IPL_DEPTH_32F)
  {
    float *e, *e_end;

    for (int v=0; v<edge->height; v++)
    {
      d1 = (short*)(dx->imageData + dx->widthStep*v);
      d2 = (short*)(dy->imageData + dy->widthStep*v);
      e = (float*)(edge->imageData + edge->widthStep*v);
      e_end = e + edge->width;
     
      for (;e!=e_end; d1++,d2++,e++)
        *e = (float)(sqrt(Sqr((float)*d1) + Sqr((float)*d2)));
    }
  }
  else throw std::runtime_error("CEdge::Norm: Unsupported image format (supported: IPL_DEPTH_16S and IPL_DEPTH_32F");
}

/**
 * Get the angle of edgels according to dx and dy
 */
void CEdge::Angle(IplImage *dx, IplImage *dy, IplImage *angle)
{
  if(!IsImage16SC1(dx) || !IsImage16SC1(dy))
    throw std::runtime_error("CEdge::Angle: Input images should be IPL_DEPTH_16S, 1 channel!");
  if(!IsImage32FC1(angle))
    throw std::runtime_error("CEdge::Angle: Output image should be IPL_DEPTH_32F, 1 channel!");
  if (!IsImageSizeEqual(dx,dy) || !IsImageSizeEqual(dx,angle))
    throw std::runtime_error("CEdge::Angle: Size of image must be equal!");

  short *d1, *d2;
  float *da, *da_end; 

  for (int v=0; v<angle->height; v++)
  {
    d1 = (short*)(dx->imageData + dx->widthStep*v);
    d2 = (short*)(dy->imageData + dy->widthStep*v);
    da = (float*)(angle->imageData + angle->widthStep*v);
    da_end = da + angle->width;

    for (;da!=da_end; d1++,d2++,da++)
      *da = (float)(atan2((float)*d2,(float)*d1));
  }
}

/**
 * Canny edge detection
 */
void CEdge::Canny(IplImage *indx, IplImage *indy, IplImage *idst, double lowThr, double highThr)
{
  if(!IsImage16SC1(indx) || !IsImage16SC1(indy))
    throw std::runtime_error("CEdge::Canny: Input images should be IPL_DEPTH_16S, 1 channel!");
  if(!IsImage8UC1(idst))
    throw std::runtime_error("CEdge::Canny: Output image should be IPL_DEPTH_8U, 1 channel!");
  if (!IsImageSizeEqual(indx,indy) || !IsImageSizeEqual(indx,idst))
    throw std::runtime_error("CEdge::Canny: Size of images must be equal!");

  CvMat dxstub, *dx = (CvMat*)indx;
  CvMat dystub, *dy = (CvMat*)indy;
  CvMat dststub, *dst = (CvMat*)idst;

  int low, high;
  void *buffer = 0;
  int* mag_buf[3];
  uchar **stack_top, **stack_bottom = 0;
  CvSize size = cvGetSize(dx);
  int flags = apertureSize;
  uchar* map;
  int mapstep, maxsize;
  int i, j;
  CvMat mag_row;

  dx = cvGetMat( dx, &dxstub );
  dy = cvGetMat( dy, &dystub );
  dst = cvGetMat( dst, &dststub );

  if( flags & CV_CANNY_L2_GRADIENT )
  {
      Cv32suf ul, uh;
      ul.f = (float)lowThr;
      uh.f = (float)highThr;
      low = ul.i;
      high = uh.i;
  }
  else
  {
      low = cvFloor( lowThr );
      high = cvFloor( highThr );
  }
  

  buffer = cvAlloc( (size.width+2)*(size.height+2) + (size.width+2)*3*sizeof(int));
 
  mag_buf[0] = (int*)buffer;
  mag_buf[1] = mag_buf[0] + size.width + 2;
  mag_buf[2] = mag_buf[1] + size.width + 2; 

  map = (uchar*)(mag_buf[2] + size.width + 2);
  mapstep = size.width + 2;

  maxsize = MAX( 1 << 10, size.width*size.height/10 );
  stack_top = stack_bottom = (uchar**)cvAlloc( maxsize*sizeof(stack_top[0]) );

  memset( mag_buf[0], 0, (size.width+2)*sizeof(int) );
  memset( map, 1, mapstep );
  memset( map + mapstep*(size.height + 1), 1, mapstep );

  /* sector numbers 
     (Top-Left Origin)

      1   2   3
       *  *  * 
        * * *  
      0*******0
        * * *  
       *  *  * 
      3   2   1
  */

  #define CE_CANNY_PUSH(d)    *(d) = (uchar)2, *stack_top++ = (d)
  #define CE_CANNY_POP(d)     (d) = *--stack_top

  mag_row = cvMat( 1, size.width, CV_32F );

  // calculate magnitude and angle of gradient, perform non-maxima supression.
  // fill the map with one of the following values:
  //   0 - the pixel might belong to an edge
  //   1 - the pixel can not belong to an edge
  //   2 - the pixel does belong to an edge
  for( i = 0; i <= size.height; i++ )
  {
      int* _mag = mag_buf[(i > 0) + 1] + 1;
      float* _magf = (float*)_mag;
      const short* _dx = (short*)(dx->data.ptr + dx->step*i);
      const short* _dy = (short*)(dy->data.ptr + dy->step*i);
      uchar* _map;
      int x, y;
      int magstep1, magstep2;
      int prev_flag = 0;

      if( i < size.height )
      {
          _mag[-1] = _mag[size.width] = 0;

          if( !(flags & CV_CANNY_L2_GRADIENT) )
              for( j = 0; j < size.width; j++ )
                  _mag[j] = abs(_dx[j]) + abs(_dy[j]);
          else
          {
              for( j = 0; j < size.width; j++ )
              {
                  x = _dx[j]; y = _dy[j];
                  _magf[j] = (float)sqrt((double)x*x + (double)y*y);
              }
          }
      }
      else
          memset( _mag-1, 0, (size.width + 2)*sizeof(int) );

      // at the very beginning we do not have a complete ring
      // buffer of 3 magnitude rows for non-maxima suppression
      if( i == 0 )
          continue;

      _map = map + mapstep*i + 1;
      _map[-1] = _map[size.width] = 1;

      _mag = mag_buf[1] + 1; // take the central row
      _dx = (short*)(dx->data.ptr + dx->step*(i-1));
      _dy = (short*)(dy->data.ptr + dy->step*(i-1));

      magstep1 = (int)(mag_buf[2] - mag_buf[1]);
      magstep2 = (int)(mag_buf[0] - mag_buf[1]);

      if( (stack_top - stack_bottom) + size.width > maxsize )
      {
          uchar** new_stack_bottom;
          maxsize = MAX( maxsize * 3/2, maxsize + size.width );
          new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0])) ;
          memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
          stack_top = new_stack_bottom + (stack_top - stack_bottom);
          cvFree( &stack_bottom );
          stack_bottom = new_stack_bottom;
      }

      for( j = 0; j < size.width; j++ )
      {
          #define CE_CANNY_SHIFT 15
          #define CE_TG22  (int)(0.4142135623730950488016887242097*(1<<CE_CANNY_SHIFT) + 0.5)

          x = _dx[j];
          y = _dy[j];
          int s = x ^ y;
          int m = _mag[j];

          x = abs(x);
          y = abs(y);
          if( m > low )
          {
              int tg22x = x * CE_TG22;
              int tg67x = tg22x + ((x + x) << CE_CANNY_SHIFT);

              y <<= CE_CANNY_SHIFT;

              if( y < tg22x )
              {
                  if( m > _mag[j-1] && m >= _mag[j+1] )
                  {
                      if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                      {
                          CE_CANNY_PUSH( _map + j );
                          prev_flag = 1;
                      }
                      else
                          _map[j] = (uchar)0;
                      continue;
                  }
              }
              else if( y > tg67x )
              {
                  if( m > _mag[j+magstep2] && m >= _mag[j+magstep1] )
                  {
                      if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                      {
                          CE_CANNY_PUSH( _map + j );
                          prev_flag = 1;
                      }
                      else
                          _map[j] = (uchar)0;
                      continue;
                  }
              }
             else
              {
                  s = s < 0 ? -1 : 1;
                  if( m > _mag[j+magstep2-s] && m > _mag[j+magstep1+s] )
                  {
                      if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                      {
                          CE_CANNY_PUSH( _map + j );
                          prev_flag = 1;
                      }
                      else
                          _map[j] = (uchar)0;
                      continue;
                  }
              }
          }
          prev_flag = 0;
          _map[j] = (uchar)1;
      }

      // scroll the ring buffer
      _mag = mag_buf[0];
      mag_buf[0] = mag_buf[1];
      mag_buf[1] = mag_buf[2];
      mag_buf[2] = _mag;
  }

 // now track the edges (hysteresis thresholding)
  while( stack_top > stack_bottom )
  {
      uchar* m;
      if( (stack_top - stack_bottom) + 8 > maxsize )
      {
          uchar** new_stack_bottom;
          maxsize = MAX( maxsize * 3/2, maxsize + 8 );
          new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0])) ;
          memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
          stack_top = new_stack_bottom + (stack_top - stack_bottom);
          cvFree( &stack_bottom );
          stack_bottom = new_stack_bottom;
      }

      CE_CANNY_POP(m);

      if( !m[-1] )
          CE_CANNY_PUSH( m - 1 );
      if( !m[1] )
          CE_CANNY_PUSH( m + 1 );
      if( !m[-mapstep-1] )
          CE_CANNY_PUSH( m - mapstep - 1 );
      if( !m[-mapstep] )
          CE_CANNY_PUSH( m - mapstep );
      if( !m[-mapstep+1] )
          CE_CANNY_PUSH( m - mapstep + 1 );
      if( !m[mapstep-1] )
          CE_CANNY_PUSH( m + mapstep - 1 );
      if( !m[mapstep] )
          CE_CANNY_PUSH( m + mapstep );
      if( !m[mapstep+1] )
          CE_CANNY_PUSH( m + mapstep + 1 );
  }

  // the final pass, form the final image
  for( i = 0; i < size.height; i++ )
  {
      const uchar* _map = map + mapstep*(i+1) + 1;
      uchar* _dst = dst->data.ptr + dst->step*i;

      for( j = 0; j < size.width; j++ )
          _dst[j] = (uchar)-(_map[j] >> 1);
  }

  cvFree( &buffer );
  cvFree( &stack_bottom );
}

/**
 * Canny edge detection
 */
void CEdge::Canny(IplImage *src, IplImage *dst, double lowThr, double highThr)
{
  IplImage *dx = cvCreateImage(cvGetSize(src), IPL_DEPTH_16S, 1 ); 
  IplImage *dy = cvCreateImage(cvGetSize(src), IPL_DEPTH_16S, 1 ); 

  Sobel(src, dx, dy);
  Canny(dx, dy, dst, lowThr, highThr);    

  cvReleaseImage(&dx);
  cvReleaseImage(&dy);
}

/**
 * Link canny edgels to segments
 * It's still simple :-)
 */
void CEdge::LinkEdge(IplImage *src, IplImage *dx, IplImage *dy, Array<Array<Edgel> *> &segments)
{
  if(!IsImage16SC1(dx) && !IsImage16SC1(dy))
    throw std::runtime_error("CEdge::LinkEdge: Gradient images should be IPL_DEPTH_16S, 1 channel!");
  if(!IsImage8UC1(src))
    throw std::runtime_error("CEdge::LinkEdge: Canny image should be IPL_DEPTH_8U, 1 channel!");
  if(!IsImageSizeEqual(dx,dy) || !IsImageSizeEqual(dx,src))
    throw std::runtime_error("CEdge::LinkEdge: Input image must have same size!");
  
  IplImage *tmp = cvCreateImage(cvGetSize(dx), IPL_DEPTH_8U, 1 );
  Array<Edgel> *arr;
  Array<Edgel> arr2;
  cvZero(tmp);

  for (int v=1; v<src->height-1; v++)
  {
    for (int u=1; u<src->width-1; u++)
    {
      if (GetPx8UC1(src,u,v)==255 && GetPx8UC1(tmp,u,v)==0)
      {
        arr2.Clear();
        arr = new Array<Edgel>;
        TraceEdge2(src, dx, dy, tmp, u, v, arr2);

        for (int i = arr2.Size()-1; i>1; i--)
          arr->PushBack(arr2[i]);
  
        TraceEdge2(src, dx, dy, tmp, u, v, *arr);
        segments.PushBack(arr);
      }
    }
  }

  cvReleaseImage(&tmp);
}






}

