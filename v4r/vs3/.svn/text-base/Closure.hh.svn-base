/**
 * @file Closure.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt Closure.
 **/

#ifndef Z_CLOSURE_HH
#define Z_CLOSURE_HH

#include <set>
#include <map>
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Line;																	/// TODO Braucht man die hier?
class Collinearity;
class LJunction;

/**
 * @brief Lines are in counter-clockwise order.\n
 * jct i is the jct between line i-1 and line i.
 */
class Closure : public Gestalt
{
public:

  // Notes:
  // The closure has two arrays jcts and colls of same size. If a
  // junction between two consecutive lines is an L-jct, jct[i] points to the
  // according L-jct and coll[i] == 0. Otherwise jct[i] == 0 and coll[i] points
  // to the respective collinearity.

  Array<LJunction*> jcts;
  Array<Collinearity*> colls;
  Array<Line*> lines;
  Array<int> senses;
  map<Closure*, double> neighbors_above;
  map<Closure*, double> neighbors_below;
  map<Closure*, double> neighbors_inside;
  map<Closure*, double> neighbors_outside;
  map<Closure*, double> neighbors_aside;
  set<Closure*> neighbors;
  double stability;
  double energy;
  int label;

  Closure(VisionCore *c);
  void FindTNeighbors();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  unsigned NumLJunctions();
  unsigned NumCollinearities();
  unsigned NumLines() {return lines.Size();}
  double SumGaps();
  double SumLines();
  double Circumference();
  void CalculateSignificance();
  Vector2 GetVertex(unsigned i);
  double GetJunctionSig(unsigned i);
  bool Inside(Vector2 p);
  double Area();
};

inline Array<Gestalt*>& Closures(VisionCore *core)
{
  return core->Gestalts(Gestalt::CLOSURE);
}
inline Closure* Closures(VisionCore *core, unsigned id)
{
  return (Closure*)core->Gestalts(Gestalt::CLOSURE, id);
}
inline unsigned NumClosures(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CLOSURE);
}

}

#endif

