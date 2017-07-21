/**
 * @file FormCubes.cc
 * @author Andreas Richtsfeld
 * @date 2008, 2010
 * @version 0.1
 * @brief Class of Gestalt principle cube.
 **/

#include "math.h"								/// TODO Für was math.h?
#include "FormCubes.hh"
#include <cstdio>

//#define CUBE_MIN_SIGNIFICANCE = 500.


namespace Z
{
/**
 * @brief Compare function for cubes.
 * @param a First Gestalt to compare.
 * @param b Second Gestalt to compare.
 */
static int CmpCubes(const void *a, const void *b)
{
  if( (*(Cube**)a)->sig > (*(Cube**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Rank cubes.
 */
void FormCubes::Rank()
{
  RankGestalts(Gestalt::CUBE, CmpCubes);
}

/**
 * @brief Mask cubes which have the center point inside the cube radius of another cube with higher significance.
 */
void FormCubes::Mask()										
{
  for(unsigned i=0; i<NumCubes(core); i++)
		for(unsigned j=0; j<NumCubes(core); j++)
			if(!Cubes(core, i)->IsMasked() && !Cubes(core, j)->IsMasked())
				if(Cubes(core, i)->sig < Cubes(core, j)->sig)
					if(Cubes(core, i)->IsInside(j))
						Cubes(core, i)->Mask(j);
  for(unsigned i=0; i<NumCubes(core); i++)
		if(Cubes(core, i)->sig < CUBE_MIN_SIGNIFICANCE)
			Cubes(core, i)->Mask(1000);
}


/**
 * @brief Reset FormCubes class.
 */
void FormCubes::Reset()
{
}


/**
 * @brief Constructor of FormCubes class.
 * @param vc Vision core
 */
FormCubes::FormCubes(VisionCore *vc) : GestaltPrinciple(vc)
{
}

/**
 * @brief Informs vision core, if principle needs operate.
 * @return Returns true, if FormCubes needs operate call (non-incremental)
 */
bool FormCubes::NeedsOperate()
{
  return false;
}


/**
 * @brief InformNewGestalt: receive new Gestalts from vision core.
 * @param type Gestalt type.
 * @param idx Index of new Gestalt.
 */
void FormCubes::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
  switch(type)
  {
		case Gestalt::FLAP_ARI:
			Create(idx);
			break;
    default:
			printf("FormCubes::InformNewGestalt: received unknown Gestalt.\n");
      break;
  }
  Rank();
  Mask();
	StopRunTime();
}


/**
 * @brief Try to create new cubes.
 * @param flap Id of flap
 */
void FormCubes::Create(unsigned flap)
{
  CreateFromFlaps(flap);
}


/**
 * @brief Create cube, based on a combination of three flaps.
 * Find three flaps, which share exactly three rectangles.
 * @param flap ID of new flap
 * @return Returns true, if new flap detected.
 */
bool FormCubes::CreateFromFlaps(unsigned f)
{
	// the 3 flaps to find
	FlapAri *flap[3];
	flap[0] = FlapsAri(core, f);

  // get every flap
  for (unsigned i=0; i<(NumFlapsAri(core)-1); i++)
  {
		Rectangle *foundRects[2];
		Rectangle *r[2];
		r[0] = FlapsAri(core, i)->rectangle[0];
		r[1] = FlapsAri(core, i)->rectangle[1];

		// only one of the rectangles of the flap is equal
		bool found = false;
		if (r[0]->ID() == flap[0]->rectangle[0]->ID() && r[1]->ID() != flap[0]->rectangle[1]->ID())
		{
			found = true;
			foundRects[0] = flap[0]->rectangle[1];
			foundRects[1] = r[1];
		}
		if (r[0]->ID() == flap[0]->rectangle[1]->ID() && r[1]->ID() != flap[0]->rectangle[0]->ID())
		{
			found = true;
			foundRects[0] = flap[0]->rectangle[0];
			foundRects[1] = r[1];
		}
		if (r[1]->ID() == flap[0]->rectangle[0]->ID() && r[0]->ID() != flap[0]->rectangle[1]->ID())
		{
			found = true;
			foundRects[0] = flap[0]->rectangle[1];
			foundRects[1] = r[0];
		}
		if (r[1]->ID() == flap[0]->rectangle[1]->ID() && r[0]->ID() != flap[0]->rectangle[0]->ID())
		{
			found = true;
			foundRects[0] = flap[0]->rectangle[0];
			foundRects[1] = r[0];
		}

		// find a third flap with the two other rectangles
		if (found)
		{
			for (unsigned j=0; j<(NumFlapsAri(core)-1); j++)
			{
				Rectangle *thirdRects[2];
				thirdRects[0] = FlapsAri(core, j)->rectangle[0];
				thirdRects[1] = FlapsAri(core, j)->rectangle[1];
					
				if (thirdRects[0] == foundRects[0] && thirdRects[1] == foundRects[1])
				{
					flap[1] = FlapsAri(core, i);
					flap[2] = FlapsAri(core, j);
					core->NewGestalt(GestaltPrinciple::FORM_CLOSURES, new Cube(core, flap));	
					return true;
				}
				else if (thirdRects[0] == foundRects[1] && thirdRects[1] == foundRects[0])
				{
					flap[1] = FlapsAri(core, i);
					flap[2] = FlapsAri(core, j);
					core->NewGestalt(GestaltPrinciple::FORM_CLOSURES, new Cube(core, flap));	
					return true;
				}
			}
		}
  }
  return false;
}

}
