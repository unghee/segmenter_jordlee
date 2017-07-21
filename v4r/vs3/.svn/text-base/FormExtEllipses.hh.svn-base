/**
 * @file FormExtEllipses.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt principle class FormExtEllipses.
 */

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Gestalt principle class FormExtEllipses.
 */
class FormExtEllipses : public GestaltPrinciple
{
private:
  bool needsOperate;
	
  void Rank();
  void Create();

public:
  FormExtEllipses(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
  void InformNewEJunction(unsigned idx);
  void NewExtEllipse(unsigned eJct);
  void ExtendExtEllipse(unsigned eJct);

};

}
