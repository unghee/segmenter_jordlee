/**
 * @file FlapAri.hh
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Header file of Gestalt FlapAri: Form flaps from rectangles.
 **/

#ifndef Z_FLAP_ARI_HH
#define Z_FLAP_ARI_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Rectangle.hh"

namespace Z
{

/**
 * @class FlapAri Flaps, created from rectangles
 * @brief Flaps, created from rectangles.
 */
class FlapAri : public Gestalt
{
public:
  Rectangle *rectangle[2];                      ///< Rectangles of the flap
  Array<Line*> sharedLines;                     ///< SharedLines from the two rectangles

  // The intersections of both rectangles are ordered clockwise. The shared line(s) are always between 0 and 3
  Vector2 orderedIsctR0[4];                     ///< Ordered intersection points of first rectangle (clockwise)
  Vector2 orderedIsctR1[4];                     ///< Ordered intersection points of second rectangle (clockwise)

  // The six intersection points starting with the "inner intersection point" from both rectangles,
  // then following all other intersection points clockwise. (0,1,2,3 is than the first rectangle,
  // 3,4,5,0 the second one)
  Vector2 isct[6];                              ///< Intersection points of the flap

  double meanGap;                               ///< Mean value of smallest two gaps between two rectangle-corners for significance
  Vector2 center;                               ///< Center point of the flap
  double radius;                                ///< Maximum radius from center to outerJcts->iscts


  FlapAri(VisionCore *c, Rectangle *r[2], double gap, Array<Line*> sLines, Vector2 oIsctR0[4], Vector2 oIsctR1[4]);
  void OrderIntersections(Rectangle *r[2], Vector2 oIR0[4], Vector2 oIR1[4]);
  void CalculateSignificance();
  bool IsInside(unsigned flap);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};


inline Array<Gestalt*>& FlapsAri(VisionCore *core)
{
  return core->Gestalts(Gestalt::FLAP_ARI);
}
inline FlapAri* FlapsAri(VisionCore *core, unsigned id)
{
  return (FlapAri*)core->Gestalts(Gestalt::FLAP_ARI, id);
}
inline unsigned NumFlapsAri(VisionCore *core)
{
  return core->NumGestalts(Gestalt::FLAP_ARI);
}
}

#endif
