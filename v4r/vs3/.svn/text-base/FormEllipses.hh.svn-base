/**
 * @file FormEllipses.hh
 * @author Andreas Richtsfeld
 * @date December 2009
 * @version 0.1
 * @brief Form ellipses from formed convex arc groups.
 *
 * TODO - try computing size just from number of edgels of arcs (minus edgels
 * TODO   grabbed from lines), evtl. again weighted by amount of arc on ellipse
 */

#ifndef Z_FORM_ELLIPSES_HH
#define Z_FORM_ELLIPSES_HH

#include "GestaltPrinciple.hh"
#include "Segment.hh"

namespace Z
{

class FormEllipses : public GestaltPrinciple
{
private:
  void Rank();
  void Mask();
  void HaveNewArcGroup(unsigned idx);

public:
  FormEllipses(VisionCore *vc);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif

