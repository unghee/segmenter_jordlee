/**
 * @file Arc.cc
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt Arc.
 */

#ifndef Z_ARC_HH
#define Z_ARC_HH

#include "Vector.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"

namespace Z
{
class AJunction;
class Segment;

/**
 * @brief Class of a circular arc.
 */
class Arc : public Gestalt
{
private:
  bool OrderAnglesCounterClockwise(double &a, double &b, double &c);
  void CalculateParams();
  void CalculateAngles();
  void CalculateColors();
  void CalculateColorHistograms();
  void CalculateTangents();
  void CalculateSignificance();
  bool HasInsideStrong(Arc *b);
  bool HasInsideWeak(Arc *b);

public:
  static int STRICT_CONVEXITY;

  Segment *seg;      ///< the originating segment
  Arc *prev_on_seg;  ///< previous arc on the same segment
  Arc *next_on_seg;  ///< next arc on the same segment
  unsigned idx[5];   ///< index of start, end and mid point, according to pixel
                     ///< order on the segment. Note that this does not not have
                     ///< to be counter clockwise. In that case idx[START]
                     ///< corresponds to point[END] and vice versa!
  Vector2 point[5];  ///< points (in counter clockwise order)
                     /// START, END, MID, ONE_THIRD, TWO_THIRD
  Vector2 norm[3];      ///< normals pointing inwards at START, END, MID
  Vector2 center;       ///< circle center
  double radius;        ///< circle radius
  double start_angle;   ///< starting angle (in counter clockwise order)
  double end_angle;     ///< end angle (in counter clockwise order)
  double angular_span;  ///< difference between start and end angle
  Vector2 tang_pt[2];   ///< tangent points at ONE_THIRD, TWO_THIRD
  Vector2 tang[2];      ///< tangents at ONE_THIRD, TWO_THIRD
  Array<AJunction*> jct[2];  ///< junctions at START, END
  RGBColor mean_col_inside;
  RGBColor mean_col_outside;

  Arc(VisionCore *vc, Segment *s, unsigned i, unsigned j, unsigned k,
      const Vector2 &cent, double rad);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void DrawSearchLines();
  unsigned NumEdgels() {return idx[END] - idx[START] + 1;}
  double Radius() {return radius;}
  double ArcLength() {return angular_span*radius;}
  bool HasInside(Arc *b);
  bool HasInside(Vector2 &p);
  bool ConvexWith(Arc *b);
  double Insideness(Arc *b);
  double Convexity(Arc *b);
  bool HasInsideWedge(Arc *b);
  bool HasInsideBeam(Arc *b);
  bool CoaxialOverlap(Arc *b);
  void AddJunction(int end, AJunction *j);
};

inline Array<Gestalt*>& Arcs(VisionCore *core)
{
  return core->Gestalts(Gestalt::ARC);
}
inline Arc* Arcs(VisionCore *core, unsigned id)
{
  return (Arc*)core->Gestalts(Gestalt::ARC, id);
}
inline unsigned NumArcs(VisionCore *core)
{
  return core->NumGestalts(Gestalt::ARC);
}

}

#endif

