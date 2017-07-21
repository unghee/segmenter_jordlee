/**
 * @file Rectangle.hh
 * @author Richtsfeld Andreas
 * @date Februar 2008, 2010
 * @version 0.1
 * @brief Header of class rectangles
 **/

// A rectangle may be created in different ways from an closure. Not all lines and l-junctions may be part of the rectangle. 
// For that reason consits a rectangle of one based closure, 4 intersection points and a array of lines.
// 
// TODO Check: We can mask rectangles only if they are from the same closure.


#ifndef Z_RECTANGLE_HH
#define Z_RECTANGLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"
#include "Closure.hh"

namespace Z
{

/**
 * @brief Class of Gestalt Rectangle
 */
class Rectangle : public Gestalt
{
private:
  void CalculateSignificance();
  double CalculateSupport();
  void GetRectPixels();
  void GetLinePixels(int x1, int y1, int x2, int y2, unsigned id);
  void SetPixel(int x, int y, unsigned id, bool neighboring);
  bool CountSupport(int x, int y);

public:
  Closure *closure;       ///< The underlying closure
  Vector2 isct[4];        ///< Intersection points in clockwise order

  unsigned *data;         ///< Image from CalculateSupport: Draw into this image to count pixels. (Bresenham alg.)
  unsigned pixelmass;     ///< Number of pixels from lines of closure
  double pixelsupport;    ///< pixelsupport of the rectangle (in %) (>100 possible?)

  double parallelity;     ///< parallelity value of the two opposed edge pairs. (see FormRectangles)
  Vector2 direction[2];   ///< mean direction of the two line-pairs (1,3 and 2,4) (should be between isct-points)
  double phi[2];          ///< mean angle of the two line-pairs

  Vector2 centerPoint;    ///< center point of the rectangle (2D mean value of corners)
  double radius;          ///< maximum distance from center-point to a corner-point
  double innerRadius;     ///< minimum distance from center-point to middle of lines.

  Rectangle(VisionCore *vc, Closure *c, Vector2 is[4], double par);
  ~Rectangle(){ delete data; };
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);

  void Init();
  bool Inside(Vector2 p);
  bool IsInside(unsigned rectangle);
  double SumGaps();
  double Area();
  double Circumference();
};


inline Array<Gestalt*>& Rectangles(VisionCore *core)
{
  return core->Gestalts(Gestalt::RECTANGLE);
}
inline Rectangle* Rectangles(VisionCore *core, unsigned id)
{
  return (Rectangle*)core->Gestalts(Gestalt::RECTANGLE, id);
}
inline unsigned NumRectangles(VisionCore *core)
{
  return core->NumGestalts(Gestalt::RECTANGLE);
}

}

#endif
