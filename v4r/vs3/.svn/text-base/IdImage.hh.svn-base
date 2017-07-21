/**
 * $Id: IdImage.hh,v 1.9 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_ID_IMAGE_HH
#define Z_ID_IMAGE_HH

#include <math.h>
#include "Vector.hh"

namespace Z
{

/**
 * Generic ID image class.
 * All grouping uses ID images to find matching groups of gestalts.
 */
class IdImage
{
private:
  bool IntersectionQuad18(int x, int y, int dy);
  bool IntersectionQuad27(int x, int y, int dx);
  bool CheckIntersectionQuad18(int x, int y, int dy, unsigned id);
  bool CheckIntersectionQuad27(int x, int y, int dx, unsigned id);

public:
  int width;
  int height;
  unsigned *data;

  IdImage(int w, int h);
  virtual ~IdImage();
  void Clear();
  unsigned Pixel(int x, int y) {return data[y*width + x];}
  virtual void SetPixel(int x, int y, unsigned id);
  virtual bool CheckIntersection(int x, int y, unsigned id);
  bool Occupied(int x, int y) {return data[y*width + x] != UNDEF_ID;}
  bool IsInside(int x, int y)
  {
    return x >= 0 && x < width && y >= 0 && y < height;
  }
  void DrawBresenhamLine(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id)
  {
    DrawBresenhamLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  void DrawBresenhamLine(int x1, int y1, int x2, int y2, unsigned id);
  void FindIntersections(const VEC::Vector2 &a, const VEC::Vector2 &b, unsigned id)
  {
    FindIntersections(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  void FindIntersections(int x1, int y1, int x2, int y2, unsigned id);
  bool CheckBresenhamLine(const VEC::Vector2 &a, const VEC::Vector2 &b)
  {
    return CheckBresenhamLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y));
  }
  bool CheckBresenhamLine(int x1, int y1, int x2, int y2);
  bool FindLineEnd(int x1, int y1, int x2, int y2, int *xend, int *yend);
};

}

#endif

