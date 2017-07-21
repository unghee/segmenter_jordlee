/**
 * @file FormCircles.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt-principle FormCircles
**/

#ifndef Z_FORM_CIRCLES_HH
#define Z_FORM_CIRCLES_HH

#include "GestaltPrinciple.hh"
#include "VisionCore.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt principle FormSpheres
 */
class FormCircles : public GestaltPrinciple
{
private:
	double minRadius;
	double roundness;

  void Rank();
	void Mask();
	void Create(unsigned ellID);

public:
	FormCircles(VisionCore *vc);
	virtual void PostOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);

};

}

#endif
