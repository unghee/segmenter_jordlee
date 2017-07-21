/**
 * @file FormCones.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt-principle FormCones
**/

#ifndef Z_FORM_CONES_HH
#define Z_FORM_CONES_HH

#include "GestaltPrinciple.hh"
#include "VisionCore.hh"
#include "EJunction.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt principle FormCones
 */
class FormCones : public GestaltPrinciple
{
private:

  void Rank();
	void Mask();

	void NewEJunction(unsigned ejct);
	void NewLJunction(unsigned ljct);
	void NewCone(EJunction *ej0, EJunction *ej1, LJunction *l);
	bool CheckGeometry(EJunction *ej0, EJunction *ej1, LJunction *l, double &diff);
	
public:
	FormCones(VisionCore *vc);
  virtual bool NeedsOperate();
	virtual void PostOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);

};

}

#endif