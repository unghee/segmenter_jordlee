/**
 * @file FormArcJunctions.hh
 * @author Zillich, Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormArcJunctions.
 **/

#ifndef Z_FORM_ARC_JUNCTIONS_HH
#define Z_FORM_ARC_JUNCTIONS_HH

#include "Arc.hh"
#include "GestaltPrinciple.hh"
#include "Line.hh"

namespace Z
{

/**
 * @brief Class FormArcJunctions, creating junctions between arcs.
 */
class FormArcJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;

  bool IsJunctionBetween(Arc *arc_i, Arc *arc_j);
  void OperateIncremental();

public:
  FormArcJunctions(VisionCore *vc);
  virtual void CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts);
};

}

#endif


