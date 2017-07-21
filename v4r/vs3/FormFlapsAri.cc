/**
 * @file FormFlapsAri.cc
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Class file of Gestalt-principle FormFlapsAri: Form flaps from rectangles, instead of closures.
 **/

#include "math.h"
#include "map"
#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "FlapAri.hh"
#include "FormFlapsAri.hh"
#include <cstdio>

namespace Z
{
	
/**
 * @brief Compare function for flaps.
 */
static int CmpFlaps(const void *a, const void *b)
{
  if( (*(FlapAri**)a)->sig > (*(FlapAri**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Rank flaps.
 */
void FormFlapsAri::Rank()
{
  RankGestalts(Gestalt::FLAP_ARI, CmpFlaps);
}


/**
 * @brief Mask flaps, if it is above another flap with higher significance.
 * Center points and radius are used to estimate if they are on top of each other.
 */
void FormFlapsAri::Mask()										
{
  for(unsigned i=0; i<NumFlapsAri(core); i++)
		for(unsigned j=0; j<NumFlapsAri(core); j++)
			if(!FlapsAri(core, i)->IsMasked() && !FlapsAri(core, j)->IsMasked())
				if(FlapsAri(core, i)->sig < FlapsAri(core, j)->sig)
					if(FlapsAri(core, i)->IsInside(j))
						FlapsAri(core, i)->Mask(j);		 
}


/**
 * @brief Constructor of FormFlapsAri
 * @param core Vision core
 */
FormFlapsAri::FormFlapsAri(VisionCore *vc) : GestaltPrinciple(vc)
{
}

/**
 * @brief Inform new Gestalt rectangle.
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormFlapsAri::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::RECTANGLE:
      CreateFlapFromRectangles(idx);		// create flaps from Rectangle
      break;
    default:
			printf("FormFlaps::InformNewGestalt: unknown Gestalt type.\n");
      break;
  }
  Rank();
  Mask();
  StopRunTime();
}


/**
 * @brief Build flaps from new informed rectangle.
 * Search line-array of rectangles for shared lines
 * @param idx Index of new informed rectangle
 */
void FormFlapsAri::CreateFlapFromRectangles(unsigned idx)
{
  Array<Line*> newRectLines = Rectangles(core, idx)->closure->lines;

  // search all rectangles, except new one
  for (unsigned i=0; i<NumRectangles(core)-1; i++)
  {
		bool alreadyFound = false;

		// get all lines of old rectangles
		Array<Line*> oldRectLines = Rectangles(core, i)->closure->lines;
		for(unsigned j=0;j<newRectLines.Size();j++)
		{
			// get every line from the closure of the old rectangle
			for(unsigned k=0;k<oldRectLines.Size();k++)
			{
				// Build new flap when shared line found an if not already a combnation found.
				if (newRectLines[j]->ID() == oldRectLines[k]->ID() && !alreadyFound)
				{

					// check existing combinations
					if (!IsExistingFlap(idx, i))											/// TODO Sollte man jetzt nicht mehr brauchen!!! (Kommt die Meldung, dass gefunden? => Sonst löschen!)
					{
						// check if the two rectangles are superposed
						bool superposed = RectanglesSuperposed(idx, i);
	
						Array<Line*> sLines;
						if (!superposed) 
						{
							sLines = GetSharedLines(idx, i);

							Vector2 orderedIsctR0[4];
							Vector2 orderedIsctR1[4];
							Rectangle *rectangle[2];
							rectangle[0] = Rectangles(core, idx);
							rectangle[1] = Rectangles(core, i);
							double meanGap = MeanGap(rectangle, orderedIsctR0, orderedIsctR1);

							alreadyFound = true;
							core->NewGestalt(GestaltPrinciple::FORM_FLAPS_ARI, 
								new FlapAri(core, rectangle, meanGap, sLines, orderedIsctR0, orderedIsctR1));
						}
					}
				}
			}
		}
  }
}



/**
 * @brief Return true, if the two rectangles are superposed. \n
 * Use the lines of the underlying closures to check if they are superposed.
 * If sign (sense) of the lines are equal, then they are superposed.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 * @return Returns true, if the rectangles are superposed.
 */
bool FormFlapsAri::RectanglesSuperposed(unsigned r0, unsigned r1)
{
	int equalSense = 0;				// number of lines with same sense
	int nonEqualSense = 0;		// number of lines without same sense
	
	Array<Line*> closLines0 = Closures(core, Rectangles(core, r0)->closure->ID())->lines;
	Array<Line*> closLines1 = Closures(core, Rectangles(core, r1)->closure->ID())->lines;

	// compare all lines
 	for(unsigned i=0;i<closLines0.Size();i++)
	{
		unsigned l0 = closLines0[i]->ID();

		for(unsigned j=0;j<closLines1.Size();j++)
		{
			unsigned l1 = closLines1[j]->ID();

			if (l0 == l1)
			{
				unsigned sense0 = Closures(core, Rectangles(core, r0)->closure->ID())->senses[i];
				unsigned sense1 = Closures(core, Rectangles(core, r1)->closure->ID())->senses[j];
				if(sense0 == sense1) equalSense++;
				else nonEqualSense++;
			}
		}	
	}
	
	// return true if rectangle is superposed
	if (equalSense == 0 && nonEqualSense >0) return false;
	else return true;
}



/**
 * @brief Returns all shared lines from two rectangles.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 */
Array<Line*> FormFlapsAri::GetSharedLines(unsigned r0, unsigned r1)
{
	Array<Line*> sLines;
  Array<Line*> closLines0 = Rectangles(core, r0)->closure->lines;
  Array<Line*> closLines1 = Rectangles(core, r1)->closure->lines;

	for(unsigned i=0;i<closLines0.Size();i++)
	{
		for(unsigned j=0;j<closLines1.Size();j++)
		{
			if (closLines0[i]->ID() == closLines1[j]->ID())
				if(!sLines.Contains(closLines0[i])) 
					sLines.PushBack(closLines0[i]);
		}
  }
  return sLines;
}


/**																																		/// TODO Diese Funktion kann nicht mehr verwendet werden, da auf L-Junctions beruht!
 * @brief Checks if the combination of rects exists already as flap.
 * @param r0 Rectangle index 0
 * @param r1 Rectangle index 1
 */
bool FormFlapsAri::IsExistingFlap(unsigned r0, unsigned r1)
{
  bool isExistingFlap = false;

  // get rectangles of every existing flap
  for (unsigned i=0; i<NumFlapsAri(core); i++)
	{
    unsigned rect0 = FlapsAri(core, i)->rectangle[0]->ID();
		unsigned rect1 = FlapsAri(core, i)->rectangle[1]->ID();

		// compare the two rectangle-pairs
		if ((rect0==r0 && rect1==r1) || (rect0==r1 && rect1==r0)) 
			isExistingFlap = true;
  }
  
if (isExistingFlap) printf("FormFlapsAri::IsExistingFlap: Das kann hier eigentlich gar nicht auftreten: Bitte sofort diese Funktion löschen!\n");

  if (isExistingFlap) return true;
  else return false;
}



/**
 * @brief Calculate the mean gap between the best combination of the intersections.
 * We know the intersection points at the corners of the rectangle. Now we are searching for \n
 * the best combination between these points and order them clockwise. \n
 * The flap intersections points are then: orderedIsctR0[0]-R1[3] and orderedIsctR0[3]-R1[0]
 * @param rect The two rectangle ids
 * @param orderedIsctR0 Intersection points are 0 and 3: ordered clockwise
 * @param orderedIsctR1 Intersection points are 0 and 3: ordered clockwise
 */
double FormFlapsAri::MeanGap(Rectangle *rectangle[2], Vector2 *orderedIsctR0, Vector2 *orderedIsctR1)
{
	double meanGap = 0.;
  Vector2 isctR0[4];					// intersection points of first rectangele
  Vector2 isctR1[4];					// intersection points of second rectangle
  double dist[4][4];					// calculated distance from corner of r0 to each corner of r1

  // get the 4 intersections of each rectangle
  for (int i=0; i<4; i++)
  {
		isctR0[i] = rectangle[0]->isct[i];
		isctR1[i] = rectangle[1]->isct[i];
  }

  // calculate distance between all intersections
  for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			dist[i][j] = Distance(isctR0[i], isctR1[j]);


// printf("##############################\n");
// printf("Distances of Rectangles: %u-%u\n", rect[0], rect[1]);
// for (int j=0; j<4; j++)
// {		
// 	printf("  %4.2f - %4.2f - %4.2f - %4.2f\n", dist[0][j], dist[1][j], dist[2][j], dist[3][j]);
// }

  // find the smallest gap from dist[4][4]
  int saveI = UNDEF_ID;
  int saveJ = UNDEF_ID;
  double minDistance = HUGE;
  for (int i=0; i<4; i++)
  {
		for (int j=0; j<4; j++)
		{
			if (dist[i][j] < minDistance)
			{
				minDistance = dist[i][j];
				saveI = i;
				saveJ = j;
			}
		}
  }
	meanGap = minDistance;

// printf("  Smallest: %u-%u\n", saveI, saveJ);

	// find the second intersection between the rectangles:
	// 2 possibilities: dist[i-1][j+1] or dist[i+1][j-1]
	double secDist[2];
	int save2I = saveI - 1;
	int save2J = saveJ + 1;
	if(save2I < 0) save2I = 3;
	if(save2J > 3) save2J = 0;
// printf("  check: %u-%u\n", save2I, save2J);
	secDist[0] = dist[save2I][save2J];
	save2I = saveI + 1;
	save2J = saveJ - 1;
	if(save2I > 3) save2I = 0;
	if(save2J < 0) save2J = 3;
// printf("  check: %u-%u\n", save2I, save2J);
	secDist[1] = dist[save2I][save2J];
// printf("=> distances: %4.2f - %4.2f\n", secDist[0], secDist[1]);

	// change start point of the rectangles
	if(secDist[0] < secDist[1]) 
		meanGap += secDist[0];
	else
	{
		saveI = saveI+1;
		if(saveI >3) saveI = 0;
		saveJ = saveJ-1;
		if(saveJ < 0) saveJ = 3;
		meanGap += secDist[1];
	}

// printf("Save: %u- %u\n", saveI, saveJ);
	
	saveJ++; // i is at left intersection, j starts at the right intersection
	if (saveJ > 3) saveJ = 0;
	for(unsigned i = 0; i<4; i++)
	{
// 		printf("    => reorder I, J: %u-%u\n", saveI, saveJ);
		orderedIsctR0[i] = isctR0[saveI];
		orderedIsctR1[i] = isctR1[saveJ];

		saveI++;
		saveJ++;
		if(saveI > 3) saveI = 0;
		if(saveJ > 3) saveJ = 0;
	}
// printf("    => second: %4.2f - %4.2f\n", secDist[0], secDist[1]);

	meanGap /= 2.;
// printf("  meanGap: %4.2f\n", meanGap);
	return meanGap;
}

}
