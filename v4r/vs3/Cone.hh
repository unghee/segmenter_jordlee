/**
 * @file Cone.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt Cone.
 **/

#ifndef Z_CONE_HH
#define Z_CONE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"
#include "EJunction.hh"

namespace Z
{

/**
 *	@brief Class, describing the Gestalt Cone.
 */
class Cone : public Gestalt
{
public:
	EJunction *ejct[2];				///< The 2 e-junctions with the same ellipse.
	LJunction *ljct;					///< The l-junction , connecting the two vertices to a cone.
	double geoSig; 						///< Significance value for cone geometry.
	
  Cone(VisionCore* vc, EJunction *e0, EJunction *e1, LJunction *l, double gS);
	bool IsInside(Cone *c);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
	virtual void CalculateSignificance();
};

inline Array<Gestalt*>& Cones(VisionCore *core)
{
  return core->Gestalts(Gestalt::CONE);
}
inline Cone* Cones(VisionCore *core, unsigned id)
{
  return (Cone*)core->Gestalts(Gestalt::CONE, id);
}
inline unsigned NumCones(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CONE);
}

}

#endif
