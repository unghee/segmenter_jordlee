/**
 * @file FormCorners.hh
 * @author Andreas Richtsfeld
 * @date September 2010
 * @version 0.1
 * @brief Header for Gestalt class of corners.
 */

#ifndef Z_CORNER_HH
#define Z_CORNER_HH

#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Math.hh"
#include "Array.hh"
#include "Vector.hh"
#include "LJunction.hh"
#include "Line.hh"
#include "Draw.hh"

namespace Z
{

class LJunction;
class Line;
	
/**
 * @brief Gestalt class of corners.
 */
class Corner : public Gestalt
{
public:
  Array<LJunction*> ljcts;                      ///< L-jcts of the corner
  Array<Line*> lines;                           ///< Lines of the corner
  Array<unsigned> near_points;                  ///< Line end nearer to the corner (START/END)
  Array<double> angle;                          ///< Angles of the arms
  Vector2 isct;                                 ///< Intersection point of the corner (is the mean of all intersection points)
  double gap;                                   ///< Gap between the intersection point and the L-Jcts
	
//   double meanLength; 	// mean of the length from the lines

  Corner(VisionCore *vc, Array<LJunction*> j, Array<Line*> l, Array<unsigned> np);
//   void AddLJunctions(Array<unsigned> &lj, Array<unsigned> &li, Array<unsigned> &np);
  void CalculateProperties();
//   void Recalc(unsigned lj);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void CalculateSignificance(double gap);
};

inline Array<Gestalt*>& Corners(VisionCore *core)
{
  return core->Gestalts(Gestalt::CORNER);
}
inline Corner* Corners(VisionCore *core, unsigned id)
{
  return (Corner*)core->Gestalts(Gestalt::CORNER, id);
}
inline unsigned NumCorners(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CORNER);
}

}

#endif
