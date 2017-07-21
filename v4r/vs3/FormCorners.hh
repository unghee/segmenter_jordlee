/**
 * @file FormCorners.hh
 * @author Andreas Richtsfeld
 * @date September 2010
 * @version 0.1
 * @brief Header for Gestalt principle class to create corners.
 * 
 * We produce only corners with 3 arms!
 */


#include "GestaltPrinciple.hh"
#include "Math.hh"
#include "Array.hh"
#include "LJunction.hh"
#include "Line.hh"
#include "Corner.hh"

namespace Z
{

/**
 * @brief Gestalt principle class for creating corners.
 */
class FormCorners : public GestaltPrinciple
{
private:

  void Rank();
  void Create(unsigned idx);
  void NewCorner(Array<LJunction*> ljcts, Array<Line*> lines, Array<unsigned> near_points);
  unsigned CornerExists(Array<Line*> lines, Array<unsigned> near_points);

public:
  FormCorners(VisionCore *vc);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
