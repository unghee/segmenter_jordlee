/**
 * @file EJunction.hh
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt Ellipse-Junction (EJunction).
 **/

#ifndef Z_EJUNCTION_HH
#define Z_EJUNCTION_HH

#include <cstdio>
#include <stdexcept>
#include "Vector2.hh"
#include "Gestalt.hh"
#include "FormEJunctions.hh"
#include "Draw.hh"
#include "Line.hh"
#include "VoteImage.hh"

namespace Z
{

class Ellipse;

/**
 * @brief Gestalt class for ellipse junctions (EJunction).
 */
class EJunction : public Gestalt
{
public:
	Ellipse *ellipse;							///< E-junction ellipse
	unsigned vertex; 							///< LEFT vertex between 0-180°, RIGHT vertex between 180°-360°
  Line *line;										///< E-junction line
  unsigned lineEnd;							///< line end (START/END) with ellipse intersection
	Vector2 isct;									///< intersection point of search-lines
	double gap[2]; 								///< gap between line (ellipse) and intersection point (isct)
	
	Array<Line*> colLines;				///< all lines which are connected via a collinearity
	Array<unsigned> colLinesEnd;	///< the nearer line end

  EJunction(VisionCore *vc, Line *l, Ellipse *e, unsigned lE, unsigned vtx);
  void UpdateColLines(Line* l, unsigned lE);
	void CalculateSignificance();
	bool IsLine(Line *l);
	bool IsColLine(Line *l);
	
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& EJunctions(VisionCore *core)
{
  return core->Gestalts(Gestalt::E_JUNCTION);
}
inline EJunction* EJunctions(VisionCore *core, unsigned id)
{
  return (EJunction*)core->Gestalts(Gestalt::E_JUNCTION, id);
}
inline unsigned NumEJunctions(VisionCore *core)
{
  return core->NumGestalts(Gestalt::E_JUNCTION);
}

}

#endif
