/**
 * $Id$
 */

#ifndef P_CEDGE_HH
#define P_CEDGE_HH

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdexcept>

#include "ImgUtils.hh"
#include "Math.hh"
#include "Edgel.hh"
#include "Segment.hh"

namespace Z 
{


class CEdge
{
private:
  bool useCol;
  int apertureSize;

  void SobelGrey(IplImage *img, IplImage *dx, IplImage *dy);
  void SobelCol(IplImage *img, IplImage *dx, IplImage *dy);
  void TraceEdge(IplImage *edgels, IplImage *tmp, int u, int v, Array<Edgel> &arr);
  void TraceEdge2(IplImage *edgels, IplImage *dx, IplImage *dy, IplImage *tmp, int u, int v, Array<Edgel> &arr);


  inline short Max(short a, short b, short c);
  inline bool TestEdgel(IplImage *edge, IplImage *tmp, int &x, int &y);

public:
  CEdge();
  ~CEdge();

  void Sobel(IplImage *img, IplImage *dx, IplImage *dy);
  void Norm(IplImage *dx, IplImage *dy, IplImage *edge);
  void Angle(IplImage *dx, IplImage *dy, IplImage *angle);
  void Canny(IplImage *src, IplImage *dst, double lowThr, double highThr);
  void Canny(IplImage *indx, IplImage *indy, IplImage *idst, double lowThr, double highThr);
  void LinkEdge(IplImage *src, IplImage *dx, IplImage *dy, Array<Array<Edgel> *> &segments);


  void Set(bool useColour, int ap){useCol=useColour; apertureSize=ap;}
};

/*********************** INLINE METHODES **************************/
inline short CEdge::Max(short a, short b, short c)
{
  return ( abs(a)>abs(b)?(abs(a)>abs(c)?a:c):abs(b)>abs(c)?b:c);
}

inline bool CEdge::TestEdgel(IplImage *edge, IplImage *tmp, int &x, int &y)
{
  //test 4-neighbourhood
  if (GetPx8UC1(edge,x+1,y)==255 && GetPx8UC1(tmp,x+1,y)==0)
    { x++; return true; }
  if (GetPx8UC1(edge,x,y+1)==255 && GetPx8UC1(tmp,x,y+1)==0)
    { y++; return true; }
  if (GetPx8UC1(edge,x-1,y)==255 && GetPx8UC1(tmp,x-1,y)==0)
    { x--; return true; }
  if (GetPx8UC1(edge,x,y-1)==255 && GetPx8UC1(tmp,x,y-1)==0)
    { y--; return true; }

  //test 8-neighbourhood
  if (GetPx8UC1(edge,x+1,y+1)==255 && GetPx8UC1(tmp,x+1,y+1)==0)
    { x++; y++; return true; }
  if (GetPx8UC1(edge,x+1,y-1)==255 && GetPx8UC1(tmp,x+1,y-1)==0)
    { x++; y--; return true; }
  if (GetPx8UC1(edge,x-1,y+1)==255 && GetPx8UC1(tmp,x-1,y+1)==0)
    { x--; y++; return true; }
  if (GetPx8UC1(edge,x-1,y-1)==255 && GetPx8UC1(tmp,x-1,y-1)==0)
    { x--; y--; return true; }

  return false;
}

}

#endif

