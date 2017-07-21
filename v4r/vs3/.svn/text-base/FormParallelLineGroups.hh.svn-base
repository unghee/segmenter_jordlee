/**
 * $Id: FormParallelLineGroups.hh,v 1.6 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_FORM_PARALLEL_LINE_GROUPS_HH
#define Z_FORM_PARALLEL_LINE_GROUPS_HH

#include "GestaltPrinciple.hh"

namespace Z
{

class FormParallelLineGroups : public GestaltPrinciple
{
private:
  Array<unsigned> angles;
  Array<unsigned> ranks;

  double AngularResolution();
  void Create();
  void Rank();

public:
  FormParallelLineGroups(Config *cfg);
  virtual void PostOperate();
};

}

#endif

