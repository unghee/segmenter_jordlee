/**
 * @file Cylinder.hh
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Header of Gestalt Cylinders
 **/

#ifndef Z_CYLINDER_HH
#define Z_CYLINDER_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
//#include "Ellipse.hh"
#include "EJunction.hh"

namespace Z
{

/**
 * @brief Class Gestalt Cylinder
 */
class Cylinder : public Gestalt
{
private:
	double geometrySig;												// significance value for good geometry of an cylinder (less is better)

  void CalculateSignificance();
//   void CalculateEqualVertex();
	bool CheckGeometry();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);

public:
	EJunction *ejct[2];											///< The two e-junctions.
	Vector2 vertex[2][2];										///< The connected vertices [e-jct][vertex]
	
//   bool equalVertex;											// connect equal vertices of both ellipses, if true
//   unsigned extEllipses[2];								// extended ellipses
//   Array<unsigned> sharedLines;						// sharedLines	
//  Array<unsigned> sharedLinesVertex[2];		// sharedLinesVertex[??/??]	TODO
// 	Vector2 center;													// center of the cone, estimated as point between the two ellipse-center points
//	double radius;													// maximum distance from center to ellipse-center + ellipse->b (end of cylinder)

  Cylinder(VisionCore *vc, EJunction *ej0, EJunction *ej1, double g);
//   bool IsInside(unsigned cone);
//   void AddSharedLines(Array<unsigned> &sL, Array<unsigned> *sLVtx);
  virtual void Draw(int detail = 0);
};

inline Array<Gestalt*>& Cylinders(VisionCore *core)
{
  return core->Gestalts(Gestalt::CYLINDER);
}
inline Cylinder* Cylinders(VisionCore *core, unsigned id)
{
  return (Cylinder*)core->Gestalts(Gestalt::CYLINDER, id);
}
inline unsigned NumCylinders(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CYLINDER);
}

}

#endif
