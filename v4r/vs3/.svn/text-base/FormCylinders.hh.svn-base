/**
 * @file FormCylinders.hh
 * @author Andreas Richtsfeld
 * @date December 2007, June 2010
 * @version 0.1
 * @brief Header of Gestalt-principle FormCylinders
 **/

#include "GestaltPrinciple.hh"
#include "Ellipse.hh"
#include "Line.hh"
#include "EJunction.hh"
#include "Cylinder.hh"
#include "Collinearity.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt-Principle FormCones
 */
class FormCylinders : public GestaltPrinciple
{
private:
  bool needsOperate;
	bool nonIncremental;

  void Rank();
	void Mask();
	void ProcessNonIncremental();
	void CreateNewCylinder(EJunction *ej0, EJunction *ej1);
	bool CheckGeometry(EJunction *ej0, EJunction *ej1, double &geometry);
  void Create();
  void NewEJunction(unsigned ejct);
	void GetNext(EJunction *ejct, Line *line0, unsigned lineEnd);
  void NewCollinearity(unsigned coll);
  void NewCylinder(unsigned eE0, unsigned eE1, Array<unsigned> &sL, Array<unsigned> *sLVtx);
  bool IsCylinder(Ellipse *e0,  Ellipse *e1);

public:
  FormCylinders(VisionCore *vc);
	virtual void PostOperate();
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
