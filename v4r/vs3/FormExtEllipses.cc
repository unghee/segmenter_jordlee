/**
 * @file FormExtEllipses.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt principle class FormExtEllipses.
 */

#include <math.h>
#include "Draw.hh"
#include "FormExtEllipses.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "EJunction.hh"
#include "ExtEllipse.hh"


namespace Z
{

static int CmpExtEllipses(const void *a, const void *b)
{
  if( ExtEllipses(*(unsigned*)a)->sig > ExtEllipses(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormExtEllipses::Rank()
{
  RankGestalts(Gestalt::EXTELLIPSE, CmpExtEllipses);
}

FormExtEllipses::FormExtEllipses(Config *cfg)
: GestaltPrinciple(cfg)
{
  needsOperate = false;
}

/*
*	TODO ARI: Inform new Lines or Junctions?
*/
void FormExtEllipses::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();

  switch(type)
  {
		case Gestalt::E_JUNCTION: InformNewEJunction(idx); break;
		default: break;
  }

  StopRunTime();
}


/**
 * @brief NewEJunction
 * @param eJct Index of E-Junction
 */
void FormExtEllipses::InformNewEJunction(unsigned eJct)
{
  // EJunction => ellipse, line
  unsigned ellipse = EJunctions(eJct)->ellipse;
	
  // gibt es schon eine ExtEllipse?
  if (Ellipses(ellipse)->extEllipse == UNDEF_ID) NewExtEllipse(eJct);
  else ExtendExtEllipse(eJct);
	
}

/*
**	NewExtEllipse
**
*/
void FormExtEllipses::NewExtEllipse(unsigned eJct)
{
  // new ExtEllipse
  NewGestalt(new ExtEllipse(eJct));
}

/*
**	Extend ExtEllipse
**
*/
void FormExtEllipses::ExtendExtEllipse(unsigned eJct)
{
  unsigned ellipse = EJunctions(eJct)->ellipse;
  unsigned extEll = Ellipses(ellipse)->extEllipse;
  ExtEllipses(extEll)->ExtendExtEllipse(eJct);
}

bool FormExtEllipses::NeedsOperate()
{ 
  return needsOperate;	
}

}
