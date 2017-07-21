/**
 * @file Circle.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt Circle.
 **/

#ifndef Z_CIRCLE_HH
#define Z_CIRCLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Class, describing the Gestalt Sphere.
 */
class Circle : public Gestalt
{
public:
	Ellipse *ellipse;			///< Ellipse
	double radius;				///< Radius of the circle.
	double roundness;			///< Roundness of the circle.

  Circle(VisionCore* vc, Ellipse *e, double rad, double rat);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
	virtual void CalculateSignificance(double r);
};

inline Array<Gestalt*>& Circles(VisionCore *core)
{
  return core->Gestalts(Gestalt::CIRCLE);
}
inline Circle* Circles(VisionCore *core, unsigned id)
{
  return (Circle*)core->Gestalts(Gestalt::CIRCLE, id);
}
inline unsigned NumCircles(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CIRCLE);
}

}

#endif
