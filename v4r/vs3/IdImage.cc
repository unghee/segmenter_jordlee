/**
 * $Id: IdImage.cc,v 1.11 2006/11/24 13:47:03 mxz Exp mxz $
 *
 * TODO: avoid multiple Bresenham implementations
 */

#include <stdlib.h>
#include <assert.h>
#include <algorithm>
#include "Math.hh"
#include "IdImage.hh"

namespace Z
{

IdImage::IdImage(int w, int h)
{
  width = w;
  height = h;
  data = new unsigned[width*height];
  assert(data != 0);
  Clear();
}

IdImage::~IdImage()
{
  delete[] data;
}

void IdImage::Clear()
{
  unsigned *end = data + width*height;
  for(unsigned *p = data; p < end; p++)
    *p = UNDEF_ID;
}

/**
 * Draw a 2D line into an image buffer using Bresenham.
 */
void IdImage::DrawBresenhamLine(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, err, incr, x, y;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  // octants 1,4,5,8
  if(abs(dx) > abs(dy))
  {
    if(dx < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dy >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dy = -dy;
    }
    // first octant bresenham
    err = -dx/2;
    x = x1;
    y = y1;
    SetPixel(x, y, id);
    while(x < x2-1) // <= x2  version A
    {
      //SetPixel(x, y, id);  version A
      err += dy;
      if(err >= 0)
      {
        y += incr;
        err -= dx;
      }
      x++;
      SetPixel(x, y, id);
    }
  }
  // octants 2,3,6,7
  else // abs(dx) <= abs(dy)
  {
    if(dy < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dx >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dx = -dx;
    }
    // second octant bresenham
    err = -dy/2;
    x = x1;
    y = y1;
    SetPixel(x, y, id);
    while(y < y2-1) // <= y2  version A
    {
      //SetPixel(x, y, id);  version A
      err += dx;
      if(err >= 0)
      {
        x += incr;
        err -= dy;
      }
      y++;
      SetPixel(x, y, id);
    }
  }
}

/**
 * Check whether any of the points oner the given line is set, and if so whether
 * set to a different id then the given ones.
 * Note: leaves out first and last pixel
 */
bool IdImage::CheckBresenhamLine(int x1, int y1, int x2, int y2)
{
  int dx, dy, err, incr, x, y;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return false;
  dx = x2 - x1;
  dy = y2 - y1;
  // octants 1,4,5,8
  if(abs(dx) > abs(dy))
  {
    if(dx < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dy >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dy = -dy;
    }
    // first octant bresenham
    err = -dx/2;
    x = x1;
    y = y1;
    while(x < x2-1)
    {
      err += dy;
      if(err >= 0)
      {
        y += incr;
        err -= dx;
      }
      x++;
      if(Occupied(x, y))
        return true;
    }
  }
  // octants 2,3,6,7
  else // abs(dx) <= abs(dy)
  {
    if(dy < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dx >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dx = -dx;
    }
    // second octant bresenham
    err = -dy/2;
    x = x1;
    y = y1;
    while(y < y2-1)
    {
      err += dx;
      if(err >= 0)
      {
        x += incr;
        err -= dy;
      }
      y++;
      if(Occupied(x, y))
        return true;
    }
  }
  return false;
}

/**
 * Find end of line drawn from point 1 to point 2.
 * The end of a line is reached at point 2, clip borders or when a non-empty
 * pixel (i.e. an intersection to an existing segment) is reached.
 * Note: Bresenham might swap point 1 and 2 and thus reverse drawing direction.
 * Thus consider all intersections and remember the one nearest to our original
 * starting point 1.
 * Returns false if line was clipped to 0 length.
 * Note: leaves out first and last pixel
 * TODO: actually is should be faster to remember the swap and avoid storing
 * minimum distance etc.
 * TODO: check if the algorithm can miss an intersection!
 */
bool IdImage::FindLineEnd(int x1, int y1, int x2, int y2, int *xend, int *yend)
{
  int dx, dy, err, incr, x, y, xstart, ystart, d, dmin;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return false;
  xstart = x1;
  ystart = y1;
  *xend = x2;
  *yend = y2;
  dmin = CityblockDistance(x1, y1, x2, y2);
  dx = x2 - x1;
  dy = y2 - y1;
  // octants 1,4,5,8
  if(abs(dx) > abs(dy))
  {
    if(dx < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dy >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dy = -dy;
    }
    // first octant bresenham
    err = -dx/2;
    x = x1;
    y = y1;
    // note: i don't check the first and last pixel
    while(x < x2-1)
    {
      err += dy;
      if(err >= 0)
      {
        y += incr;
        err -= dx;
      }
      x++;
      if(IntersectionQuad18(x, y, incr))
      {
        if((d = CityblockDistance(xstart, ystart, x, y)) <= dmin)
        {
          dmin = d;
          *xend = x;
          *yend = y;
        }
      }
    }
  }
  // octants 2,3,6,7
  else // abs(dx) <= abs(dy)
  {
    if(dy < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dx >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dx = -dx;
    }
    // second octant bresenham
    err = -dy/2;
    x = x1;
    y = y1;
    // note: i don't check the first and last pixel
    while(y < y2-1)
    {
      err += dx;
      if(err >= 0)
      {
        x += incr;
        err -= dy;
      }
      y++;
      if(IntersectionQuad27(x, y, incr))
      {
        if((d = CityblockDistance(xstart, ystart, x, y)) <= dmin)
        {
          dmin = d;
          *xend = x;
          *yend = y;
        }
      }
    }
  }
  return true;
}

void IdImage::FindIntersections(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, err, incr, x, y;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  // octants 1,4,5,8
  if(abs(dx) > abs(dy))
  {
    if(dx < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dy >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dy = -dy;
    }
    // first octant bresenham
    err = -dx/2;
    x = x1;
    y = y1;
    CheckIntersectionQuad18(x, y, incr, id);
    while(x < x2-1) // <= x2  version A
    {
      err += dy;
      if(err >= 0)
      {
        y += incr;
        err -= dx;
      }
      x++;
      CheckIntersectionQuad18(x, y, incr, id);
    }
  }
  // octants 2,3,6,7
  else // abs(dx) <= abs(dy)
  {
    if(dy < 0)
    {
      std::swap(x2, x1);
      std::swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dx >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dx = -dx;
    }
    // second octant bresenham
    err = -dy/2;
    x = x1;
    y = y1;
    CheckIntersectionQuad27(x, y, incr, id);
    while(y < y2-1) // <= y2  version A
    {
      err += dx;
      if(err >= 0)
      {
        x += incr;
        err -= dy;
      }
      y++;
      CheckIntersectionQuad27(x, y, incr, id);
    }
  }
}

void IdImage::SetPixel(int x, int y, unsigned id)
{
  data[y*width + x] = id;
}

/**
 * Calculate whether there is an intersection at (x,y), for quadrants 1 and 8.
 * Check area is widened by dy.
 */
bool IdImage::IntersectionQuad18(int x, int y, int dy)
{
  bool ret = Occupied(x, y);
  if(y+dy >=0 && y+dy < height)
    ret = ret || Occupied(x, y+dy);
  return ret;
}

/**
 * Calculate whether there is an intersection at (x,y), for quadrants 2 and 7.
 * Check area is widened by dx.
 */
bool IdImage::IntersectionQuad27(int x, int y, int dx)
{
  bool ret = Occupied(x, y);
  if(x+dx >=0 && x+dx < width)
    ret = ret || Occupied(x+dx, y);
  return ret;
}

bool IdImage::CheckIntersection(int x, int y, unsigned id)
{
  return data[y*width + x] != UNDEF_ID && data[y*width + x] != id;
}

bool IdImage::CheckIntersectionQuad18(int x, int y, int dy, unsigned id)
{
  return CheckIntersection(x, y, id);//TODO || CheckIntersection(x, y+dy, id);
}

bool IdImage::CheckIntersectionQuad27(int x, int y, int dx, unsigned id)
{
  return CheckIntersection(x, y, id);//TODO || CheckIntersection(x+dx, y, id);
}

}

#if 0
void gbham(int xstart,int ystart,int xend,int yend,int *npix,int **xpix,int
**ypix)
{
     int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
 
/* compute the distance in both directions */
     dx = xend - xstart;
     dy = yend - ystart;
 
/* compute the sign of the increment in both directions */
     if(dx<0)
     {
       incx = -1;
       dx = -dx;
     }
     else
       incx = dx ? 1 : 0;
     
     if(dy < 0)
     {
       incy = -1;
       dy = -dy;
     }
     else
       incy = dy ? 1 : 0;
 
/* determine which distance is larger */
     dist = (dx > dy)?dx:dy;
 
/* initilaizations before loop */
     x = xstart;
     y = ystart;
     xerr = dx;
     yerr = dy;
     
     *npix = dist + 1;
     
     *xpix = (int*)malloc( *npix * sizeof( int));
     if( NULL == *xpix)
     {
       *npix = 0;
       return;
     }
     
     *ypix=(int*)malloc( *npix * sizeof( int));
     if( NULL == *ypix)
     {
       free( *xpix);
       *npix = 0;
       return;
     }
 
/* compute the pixels */
     for(t = 0; t < dist; ++t)
     {
       (*xpix)[t] = x;
       (*ypix)[t] = y;
     
       xerr += dx;
       yerr += dy;
     
       if(xerr > dist)
       {
         xerr -= dist;
         x += incx;
       }
     
       if(yerr>dist)
       {
         yerr -= dist;
         y += incy;
       }
     }
     
     (*xpix)[dist] = xend;
     (*ypix)[dist] = yend;
} /* gbham() */

/*
// first octant only
void linev6(Screen &s,
              unsigned x1, unsigned y1,
              unsigned x2, unsigned y2,
              unsigned char colour )
    {
      int dx  = x2 - x1,
          dy  = y2 - y1,
          y   = y1,
          eps = 0;
    
      for ( int x = x1; x <= x2; x++ )  {
        s.Plot(x,y,colour);
        eps += dy;
        if ( (eps << 1) >= dx )  {
          y++;  eps -= dx;
        }
      }
    }
*/
/*
function line(x0, x1, y0, y1)
     boolean steep := abs(y1 - y0) > abs(x1 - x0)
     if steep then
         std::swap(x0, y0)
         std::swap(x1, y1)
     int deltax := abs(x1 - x0)
     int deltay := abs(y1 - y0)
     int error := 0
     int deltaerr := deltay
     int x := x0
     int y := y0
     if x0 < x1 then xstep := 1 else xstep := -1
     if y0 < y1 then ystep := 1 else ystep := -1
     if steep then plot(y,x) else plot(x,y)
     while x != x1
         x := x + xstep
         error := error + deltaerr
         if 2×error > deltax
             y := y + ystep
             error := error - deltax
         if steep then plot(y,x) else plot(x,y)
*/
/*
 function line(x0, x1, y0, y1)
     boolean steep := abs(y1 - y0) > abs(x1 - x0)
     if steep then
         std::swap(x0, y0)
         std::swap(x1, y1)
     if x0 > x1 then
         std::swap(x0, x1)
         std::swap(y0, y1)
     int deltax := x1 - x0
     int deltay := abs(y1 - y0)
     int error := 0
     int y := y0
     if y0 < y1 then ystep := 1 else ystep := -1
     for x from x0 to x1
         if steep then plot(y,x) else plot(x,y)
         error := error + deltay
         if 2×error ≥ deltax
             y := y + ystep
             error := error - deltax
*/

#endif

