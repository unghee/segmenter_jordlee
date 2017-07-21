/**
 * @file Collinearity.hh
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt class collinearity.
 */

#ifndef Z_COLLINEARITY_HH
#define Z_COLLINEARITY_HH

#include "Gestalt.hh"
#include "Vector2.hh"

namespace Z
{

class Line;

/**
 * @brief Gestalt class Collinearity
 */
class Collinearity : public Gestalt
{
private:
  void CalculateColors();
  void CalculateSignificance();

public:
	
  Line* line[2];					///< Line 0 is the longer, line 1 the shorter one
  int near_point[2];			///< The line end, which is nearer to the collinearity
  double gap;      				///< TODO: Distance between endpoints?? or sum of distances between end points and intersection?
  double gap_cor;  				///< Corrected distance if coll terminated by T-jcts
  double col_dist;				///< Colour distance
  Vector2 vertex;  				///< TODO: change name to midpoint or something

  Collinearity(VisionCore *vc, Line *line_i, Line *line_j, int end_i, int end_j);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void Recalc();
  void CorrectGap();
  int WhichLineIs(Line *l) throw(std::runtime_error);
  Line* OtherLine(Line *l);
  int WhichEndIs(Line *l); 
};

inline Array<Gestalt*>& Collinearities(VisionCore *core)
{
  return core->Gestalts(Gestalt::COLLINEARITY);
}
inline Collinearity* Collinearities(VisionCore *core, unsigned id)
{
  return (Collinearity*)core->Gestalts(Gestalt::COLLINEARITY, id);
}
inline unsigned NumCollinearities(VisionCore *core)
{
  return core->NumGestalts(Gestalt::COLLINEARITY);
}

}

#endif

