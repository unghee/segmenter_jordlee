/**
 * $Id: TJunction.hh,v 1.6 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_T_JUNCTION_HH
#define Z_T_JUNCTION_HH

#include "Vector2.hh"
#include "Gestalt.hh"

namespace Z
{

class Line;
class Collinearity;
class LJunction;

// we have a LEFT bar line, a RIGHT bar line and a MID pole line
#define POLE MID

class TJunction : public Gestalt
{
private:
  void CalculateSignificance();

public:
  Line *line[3];       ///< LEFT/RIGHT/POLE
  int lend[3][2];      ///< line ends, LEFT/RIGHT/POLE and INNER/OUTER
  Collinearity *coll;  ///< the collinearity between left and right line
  LJunction *ljct[2];  ///< the L-jcts between pole and left and right line
  Vector2 isct;        ///< intersection point
  Vector2 dir;  ///< pointing from pole inner end to intersection, i.e. to the
                // occluding surface
  double gap;

  TJunction(VisionCore *c, Line *pole, Line *left, Line *right,
      int end_p, int end_l, int end_r,
       Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r,
      double g, const Vector2 &inter);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  int WhichLineIs(Line *l) throw(std::runtime_error);
  void Recalc();
};

inline Array<Gestalt*>& TJunctions(VisionCore *core)
{
  return core->Gestalts(Gestalt::T_JUNCTION);
}
inline TJunction* TJunctions(VisionCore *core, unsigned id)
{
  return (TJunction*)core->Gestalts(Gestalt::T_JUNCTION, id);
}
inline unsigned NumTJunctions(VisionCore *core)
{
  return core->NumGestalts(Gestalt::T_JUNCTION);
}

}

#endif

