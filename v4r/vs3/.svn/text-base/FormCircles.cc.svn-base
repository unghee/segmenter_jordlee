/**
 * @file FormCircles.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt-principle FormCircles
 **/

#include "FormCircles.hh"
#include "Circle.hh"
#include "Ellipse.hh"

namespace Z
{

static int CmpCircles(const void *a, const void *b)
{
  if( (*(Circle**)a)->sig > (*(Circle**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormCircles::Rank()
{
  RankGestalts(Gestalt::CIRCLE, CmpCircles);
}

/**
 *	@brief Masking bad results of spheres
 */
void FormCircles::Mask()											// TODO masking!
{
//   for(unsigned i=0; i<NumCones(); i++)
//   {
// 		for(unsigned j=0; j<NumCones(); j++)
// 		{
// 			if(!Cones(i)->IsMasked() && !Cones(j)->IsMasked())
// 			if(Cones(i)->sig < Cones(j)->sig)
// 			{
// 				if(Cones(i)->IsInside(j))
// 				{	
// 					Cones(i)->Mask(j);		 
// 				}
// 			}		  
// 		}
//   }

	// Mask all spheres, which using a masked ellipse.
	for(unsigned i=0; i<NumCircles(core); i++)
	{
		Circle *sphere = (Circle*)core->RankedGestalts(Gestalt::CIRCLE, i);
		if(sphere->ellipse->IsMasked())
			sphere->Mask(10000);
	}
}


/**
 * @brief Constructor of Gestalt-principle FormSpheres: Creates the Gestalt Sphere from
 * the underlying Gestalts ellipses.
 */
FormCircles::FormCircles(VisionCore *vc) : GestaltPrinciple(vc)
{
	minRadius = 2.;				// minimum radius
	roundness = 0.15;			// required roundness of the sphere (arbitrary threshold)
}

/**
 * @brief Inform principle about new Gestalt.
 * @param type Type of Gestalt
 * @param idx Index of Gestalt
 */
void FormCircles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();

	if (type == Gestalt::ELLIPSE)
  	Create(idx);
	
  StopRunTime();
}

/**
 * @brief Try to create a new Gestalt sphere, when new ellipse appears.
 * Creation of shperes depends on "roundness" and minimum radius.
 * @param ellID Index of Gestalt ellipse.
 */
void FormCircles::Create(unsigned ellID)
{
	Ellipse *ell = Ellipses(core, ellID);
	double ratio = (ell->a / ell->b);
	double radius = (ell->a + ell->b)/2.;
	if (ratio >= (1-roundness) && ratio <= (1+roundness) && radius > minRadius)	
		core->NewGestalt(GestaltPrinciple::FORM_CIRCLES, new Circle(core, ell, radius, ratio));
}

/**
 * @brief Post operation procedure.
 */
void FormCircles::PostOperate()
{
	Rank();
	Mask();
}


}
