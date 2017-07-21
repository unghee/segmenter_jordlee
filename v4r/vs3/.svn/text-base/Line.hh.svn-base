/**
 * @file Line.hh
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt Line (VisibleLine).
 **/

#ifndef Z_LINE_HH
#define Z_LINE_HH

#include <list>
#include <set>
#include "Vector.hh"
#include "VisionCore.hh"
#include "Draw.hh"
#include "Segment.hh"


#include "LJunction.hh"
#include "Collinearity.hh"
#include "TJunction.hh"
#include "Closure.hh"
#include "Corner.hh"


namespace Z
{

class LJunction;    // forward declaration of some classes
class Collinearity;
class TJunction;
class Closure;
class Corner;

/**
 * @brief A directed straight line.
 */
class Line : public Gestalt
{
protected:
  RGBColor mean_col[2];                 ///< mean color LEFT/RIGHT
  void CalculateParameters();
  virtual void CalculateColors() = 0;
  void MoveJunctions(Line *l2, int end);

public:
  Vector2 point[2];                     ///< end points, START/END
  Vector2 dir;                          ///< direction of the line, normalised to 1
  Vector2 tang[2];                      ///< tangents STAR/END
  double phi;                           ///< angular direction (0-2PI)
  double len;                           ///< length of line
  //double s;                           ///< hessian form: distance
  //double theta;                       ///< hessian form: angle
  Array<Collinearity*> coll[2];         ///< collinearities at line-end [START/END]
  Array<LJunction*> l_jct[2][2];        ///< L-junctions at START/END and LEFT/RIGHT
  TJunction* t_jct[2];                  ///< T-junctions at START/END
  Array<TJunction*> pt_jct[2][2];       ///< passive T-jcts, START/END and LEFT/RIGHT
  Array<Closure*> closures;             ///< Closures TODO: have two lists, for both senses
  Array<Corner*> corners[2];            ///< Corners at [START/END]
  int label;                            ///< label LEFT/RIGHT
  double energy;                        ///< energy for global conistency
  double stability;                     ///< stability for global consistency
  std::set<Line*> neighbors;            ///< neighbors for global consistency
  Line* next;                           ///< next line if split
  Line* defer_vote;                     ///< lines created by splitting don't vote themselves
                                        /// but defer votes to original line

  unsigned idx[2];                      ///< index of start and end point
  Segment* seg;                         ///< the originating segment

protected:
  Line(VisionCore *vc);

public:
  virtual void Draw(int detail = 0);
  void DrawVotes();
  virtual void DrawInfo();
  virtual const char* GetInfo();
  double Length() {return len;}                                 ///< Return length of line
  double MinEndpointDistance(const Line* l);
  double DistanceToPoint(const Vector2 &q);
  void AddLJunction(int end, int side, LJunction* jct);
  void AddCollinearity(int end, Collinearity* co);
  void AddPassiveTJunction(int end, int side, TJunction* jct);
  RGBColor MeanCol(int side) {return mean_col[side];}           ///< Return mean color of one side.
  virtual Line* Split(const Vector2 &p) = 0;
  bool IsSplit() {return next != 0;}                            ///< Returns true, if line is splitted.
  bool HasDeferVote() {if(defer_vote != this) return true; else return false;}
};

/**
 * @brief Visible line fitted to edgels.
 */
class VisibleLine : public Line
{
private:
  void CalculateSignificance();
  virtual void CalculateColors();
  unsigned FindSplitIdx(const Vector2 &p);

public:
  VisibleLine(VisionCore *c, Segment* s, unsigned i, unsigned j);
  unsigned NumEdgels() {return idx[END] - idx[START] + 1;}       ///< Return the number of edgels
  void Recalc();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  virtual Line* Split(const Vector2 &p);
};

/**
 * @brief CotLine
 * Note: just an idea for now
 * Cotermination Line
 * Whenever a segment terminates unexplained (no T, L or C) create a
 * termination. Link two terminations with cotermination line = inferred
 * contour.
 */
class CotLine : public Line
{
};


inline Array<Gestalt*>& Lines(VisionCore *core)
{
  return core->Gestalts(Gestalt::LINE);
}
inline Line* Lines(VisionCore *core, unsigned id)
{
  return (Line*)core->Gestalts(Gestalt::LINE, id);
}
inline unsigned NumLines(VisionCore *core)
{
  return core->NumGestalts(Gestalt::LINE);
}

}

#endif

