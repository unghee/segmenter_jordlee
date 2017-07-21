/**
 * @file FormFlapsAri.hh
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Header file of Gestalt-principle FormFlapsAri: Form flaps from rectangles, instead of closures.
 **/

#ifndef Z_FORM_FLAPS_ARI_HH
#define Z_FORM_FLAPS_ARI_HH

#include "GestaltPrinciple.hh"
#include "Rectangle.hh"
#include "Line.hh"
#include "Vector.hh"

namespace Z
{

/**
 * @class FormFlapsAri Estimates flaps from neighboring rectangles.
 */
class FormFlapsAri : public GestaltPrinciple
{
private:
	void Mask();
  void Rank();
  void CreateFlapFromRectangles(unsigned idx);
  bool RectanglesSuperposed(unsigned r0, unsigned r1);
  Array<Line*> GetSharedLines(unsigned r0, unsigned r1);
  bool IsExistingFlap(unsigned r0, unsigned r1);
  double MeanGap(Rectangle *rectangle[2], Vector2 *orderedIsctR0, Vector2 *orderedIsctR1);

public:
  FormFlapsAri(VisionCore *vc);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}


#endif