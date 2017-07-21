/**
 * @file FormCubes.hh
 * @author Andreas Richtsfeld
 * @date 2008, 2010
 * @version 0.1
 * @brief Header of class FormCubes.
 **/

#include "GestaltPrinciple.hh"
#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"
#include "Cube.hh"

namespace Z
{

// TODO Threshold: Minimum of allowed significance, to build a new hypothesis.
static const double CUBE_MIN_SIGNIFICANCE = 500.;

/**
 * @brief Class of Gestalt principle FormCubes.
 */
class FormCubes : public GestaltPrinciple
{
private:

  void Rank();
  void Mask();
  void Create(unsigned flap);
  bool CreateFromFlaps(unsigned f);


public:
  FormCubes(VisionCore *vc);
  virtual void Reset();
	virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
