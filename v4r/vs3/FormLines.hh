/**
 * @file FormLines.hh
 * @author Zillich, Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormLines.
 */

#ifndef Z_FORM_LINES_HH
#define Z_FORM_LINES_HH

#include "Array.hh"
#include "Edgel.hh"
#include "GestaltPrinciple.hh"

namespace Z
{
  
class FormLines : public GestaltPrinciple
{
private:
  bool done;

  void Create();
  void Rank();

public:
  FormLines(VisionCore *vc);
  virtual void Reset();
  virtual void PreOperate();
  virtual bool NeedsOperate() {return !done;}
};

}

#endif

