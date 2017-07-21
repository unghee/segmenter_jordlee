/**
 * @file FormFlaps.hh
 * @author Michael Zillich, Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Form flaps from closures (flap-areas from polygons)
 */

#include "GestaltPrinciple.hh"
#include "Closure.hh"

namespace Z
{

/**
 * @brief Form flaps from closures. Flaps are defined here as two polygons with share one or more lines.
 */
class FormFlaps : public GestaltPrinciple
{
private:
  void HaveNewClosure(unsigned i);
  bool StartOfCollinearRun(Closure *clos_i, unsigned pos_i, Closure *clos_j, unsigned pos_j);
  bool EndOfCollinearRun(Closure *clos_i, unsigned pos_i, Closure *clos_j, unsigned pos_j);
  void NewFlap(Closure *clos_i, Closure *clos_j);
  void Mask();

public:
  FormFlaps(VisionCore *vc);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

