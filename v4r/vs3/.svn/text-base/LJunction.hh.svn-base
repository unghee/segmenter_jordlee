/**
 * @file LJunction.hh
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Gestalt class for L-junctions.
 **/

#ifndef Z_L_JUNCTION_HH
#define Z_L_JUNCTION_HH

#include "Vector2.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Array.hh"
#include "Draw.hh"
#include "Corner.hh"
#include "Line.hh"
#include "Vector.hh"

namespace Z
{

class Line;
class Corner;

/**
 * @brief Gestalt class LJunction
 */
class LJunction : public Gestalt
{
private:
  void CalculateSignificance();
  void PrintSig();

public:
  Line *line[2];     ///< left and right arm					/// TODO TODO ordered? left/right?
  double sense[2];   ///< whether line dirs point to or from junction
  int near_point[2]; ///< which point of each line is nearer to the jct
  Vector2 dir[2];    ///< directions of arms, pointing outwards
  double dist[2];    ///< distance to left and right endpoints
  Vector2 isct;      ///< intersection point
  Vector2 dir_i;     ///< direction points inwards, bisects angle
  double r;          ///< distance between endpoints
  double col_dist;   ///< color distance between inside mean colors

  LJunction(VisionCore *c, Line *line_i, Line *line_j, int end_i, int end_j, const Vector2 &inter);
  void Recalc();
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  double Gap(int side);
  double OpeningAngle();
};

extern bool Inside(LJunction *a, LJunction *b);
extern bool Opposing(LJunction *a, LJunction *b);

inline Array<Gestalt*>& LJunctions(VisionCore *core)
{
  return core->Gestalts(Gestalt::L_JUNCTION);
}
inline LJunction* LJunctions(VisionCore *core, unsigned id)
{
  return (LJunction*)core->Gestalts(Gestalt::L_JUNCTION, id);
}
inline unsigned NumLJunctions(VisionCore *core)
{
  return core->NumGestalts(Gestalt::L_JUNCTION);
}

}

#endif

