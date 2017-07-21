/**
 * @file FlapAri.hh
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Class file of Gestalt FlapAri: Flaps from rectangles.
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"
#include <cstdio>

namespace Z
{

/**
 * @brief Constructor of Gestalt class FlapAri.
 * @param c Vision core
 * @param r Rectangles
 * @param mG Mean gap between rectangle corners (shared line)
 * @param sL Shared lines between the two rectangles.
 * @param oIsctR0 Ordered intersections from first rectangle. \n
 * The intersections of both rectangles are ordered clockwise. The shared line(s) are always between 0 and 3
 * @param oIsctR1 Ordered intersections from second rectangle
 */
FlapAri::FlapAri(VisionCore *c, Rectangle *r[2], double mG, Array<Line*> sLines,
								 Vector2 oIsctR0[4], Vector2 oIsctR1[4]) : Gestalt(c, FLAP_ARI)
{
	OrderIntersections(r, oIsctR0, oIsctR1);
  meanGap = mG;
	sharedLines = sLines;

	// calculate center and radius
	center = 	(orderedIsctR0[0] + orderedIsctR0[1] + orderedIsctR1[0] + orderedIsctR1[1])/4.;
	double rad0 = (orderedIsctR0[2] - center).Length();
  double rad1 = (orderedIsctR0[3] - center).Length();
  double rad2 = (orderedIsctR1[2] - center).Length();
  double rad3 = (orderedIsctR1[3] - center).Length();
  radius = Max(rad0, rad1);
  radius = Max(radius, rad2);
  radius = Max(radius, rad3);  

  CalculateSignificance();
}

/**
 * @brief Order the intersection points from the rectangles. \n
 * TODO Beschreibung
 * @param r The two rectangles
 * @param oIR0 The ordered intersection points of the first rectangle.
 * @param oIR1 The ordered intersection points of the second rectangle.
 */
void FlapAri::OrderIntersections(Rectangle *r[2], Vector2 oIR0[4], Vector2 oIR1[4])
{
	// Calculate Vectors
	Vector2 vR032 = oIR0[3] - oIR0[2];
	Vector2 vR101 = oIR1[0] - oIR1[1];
	Vector2 vR001 = oIR0[0] - oIR0[1];
	Vector2 vR132 = oIR1[3] - oIR1[2];

	if(LeftOf(vR032, vR101) && LeftOf(vR001, vR132))		// 1st: 3-0 inner && 0-3 outer: change r0-r1
	{
		isct[0] = ((oIR0[3] + oIR1[0])/2.);
		isct[1] = oIR1[1];
		isct[2] = oIR1[2];
		isct[3] = ((oIR0[0] + oIR1[3])/2.);
		isct[4] = oIR0[1];
		isct[5] = oIR0[2];

		rectangle[1] = r[0];
		rectangle[0] = r[1];
		for (int i=0; i<4; i++)
		{
			orderedIsctR0[i] = oIR1[i];
			orderedIsctR1[i] = oIR0[i];
		}
	}
	else // if(LeftOf(vR001, vR132) && LeftOf(vR032, vR112))	// 1st: 0-3 inner && 3-0 outer: no change
	{
		isct[0] = ((oIR0[0] + oIR1[3])/2.);
		isct[1] = oIR0[1];
		isct[2] = oIR0[2];
		isct[3] = ((oIR0[3] + oIR1[0])/2.);
		isct[4] = oIR1[1];
		isct[5] = oIR1[2];

		rectangle[0] = r[0];
		rectangle[1] = r[1];
		for (int i=0; i<4; i++)
		{
			orderedIsctR0[i] = oIR0[i];
			orderedIsctR1[i] = oIR1[i];
		}
	}

//	We do not order the intersections, when the flap is not bent to one direction.
// 	if((LeftOf(vR032, vR101) && !LeftOf(vR001, vR132)) || (!LeftOf(vR032, vR101) && LeftOf(vR001, vR132)))
// 	{
// 		printf("  Flap %u is not bent to one direction: rect: %u-%u!\n", ID(), rectangle[0]->ID(), rectangle[1]->ID());
// 	}
}


/**
 *	@brief Returns true, if both rectangle center points are inside the two different rectangle radius.
 *	@param flap Index of Flap to compare.
 */
bool FlapAri::IsInside(unsigned flap)
{
	if((((FlapsAri(core, flap)->rectangle[0]->centerPoint - rectangle[0]->centerPoint).Length() < rectangle[0]->innerRadius) &&
		 ((FlapsAri(core, flap)->rectangle[1]->centerPoint - rectangle[1]->centerPoint).Length() < rectangle[1]->innerRadius)) ||
		 (((FlapsAri(core, flap)->rectangle[0]->centerPoint - rectangle[1]->centerPoint).Length() < rectangle[0]->innerRadius) &&
		 ((FlapsAri(core, flap)->rectangle[1]->centerPoint - rectangle[0]->centerPoint).Length() < rectangle[1]->innerRadius))) return true;	
	else return false;
}


/**
 *	@brief Draws the Gestalt Flap to the main draw area.
 *	@param detail Degree of detail.
 */
void FlapAri::Draw(int detail)
{
	// show flap, based on ordered flap junctions
  if(detail == 0)
  {
    for(unsigned i = 0; i < 6; i++)
      DrawLine2D(isct[i].x, isct[i].y, isct[(i<5?i+1:0)].x, isct[(i<5?i+1:0)].y);
		DrawLine2D( isct[0].x, isct[0].y, isct[3].x, isct[3].y);
	}
  else if(detail == 1 || detail == 4)
	{
    for(unsigned i = 0; i < 6; i++)
      DrawLine2D(isct[i].x, isct[i].y, isct[(i<5?i+1:0)].x, isct[(i<5?i+1:0)].y, RGBColor::yellow);
		DrawLine2D( isct[0].x, isct[0].y, isct[3].x, isct[3].y, RGBColor::yellow);
	}

	// show flap, based on ordered rectangle junctions
  if(detail == 2 || detail == 3 || detail == 5)
  {
    for(unsigned i = 0; i < 4; i++)
		{
      DrawLine2D(
				orderedIsctR0[i].x, orderedIsctR0[i].y,
				orderedIsctR0[(i<3?i+1:0)].x, orderedIsctR0[(i<3?i+1:0)].y, RGBColor::yellow);
      DrawLine2D(
				orderedIsctR1[i].x, orderedIsctR1[i].y,
				orderedIsctR1[(i<3?i+1:0)].x, orderedIsctR1[(i<3?i+1:0)].y, RGBColor::yellow);
    }
  }

	// show flap and shared lines
  if(detail == 1)
		for(unsigned i=0; i<sharedLines.Size(); i++)
		{
			sharedLines[i]->Draw(detail);
		}	

	// show rectangles with both rectangle-numbers
  if(detail == 2)
	{
		rectangle[0]->Draw(0);
		rectangle[1]->Draw(0);
		char rectID[5];
		snprintf(rectID, 5, "%u", rectangle[0]->ID());
		DrawText2D(rectID, rectangle[0]->centerPoint.x, rectangle[0]->centerPoint.y, RGBColor::blue);
		snprintf(rectID, 5, "%u", rectangle[1]->ID());
		DrawText2D(rectID, rectangle[1]->centerPoint.x, rectangle[1]->centerPoint.y, RGBColor::red);
	}

	// show flap with the ordered intersections from both rectangles (orderedIsctRx)
  if(detail == 3)
	{
		for(unsigned i=0; i<4; i++)
		{
			char isctNr[5];
			snprintf(isctNr, 5, "%u", i);
			DrawText2D(isctNr, orderedIsctR0[i].x -3, orderedIsctR0[i].y +8, RGBColor::blue);
			snprintf(isctNr, 5, "%u", i);
			DrawText2D(isctNr, orderedIsctR1[i].x -3, orderedIsctR1[i].y -8, RGBColor::red);
		}
	}	

	// show flap with the intersection points (isct)
  if(detail == 4)
	{
		for(unsigned i=0; i<6; i++)
		{
			char isctNr[5];
			snprintf(isctNr, 5, "%u", i);
			DrawText2D(isctNr, isct[i].x -3, isct[i].y -8, RGBColor::red);
		}
	}	
	
	// show flap with center-point and radius
	if(detail == 5)
	{
		// Draw center point and radius
		FillEllipse2D(rectangle[0]->centerPoint.x, rectangle[0]->centerPoint.y, 2, 2, 0, RGBColor::white);
		FillEllipse2D(rectangle[1]->centerPoint.x, rectangle[1]->centerPoint.y, 2, 2, 0, RGBColor::white);

		DrawEllipse2D(rectangle[0]->centerPoint.x, rectangle[0]->centerPoint.y, rectangle[0]->innerRadius, rectangle[0]->innerRadius, 0, RGBColor::white);
		DrawEllipse2D(rectangle[1]->centerPoint.x, rectangle[1]->centerPoint.y, rectangle[1]->innerRadius, rectangle[1]->innerRadius, 0, RGBColor::white);
	}

	// show rectangle properties
  if(detail > 5)
  {
  	rectangle[0]->Draw(detail-6);
		rectangle[1]->Draw(detail-6);
  }
}

/**
 * @brief Returns all information about the Gestalt.
 * @return Returns all information about the Gestalt.
 */
const char* FlapAri::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
	
  n += snprintf(info_text, info_size, "%s  rects: %u - %u\n  shared lines:",
      Gestalt::GetInfo(), rectangle[0]->ID(), rectangle[1]->ID());

	for(unsigned i=0; i<sharedLines.Size(); i++)
  	n += snprintf(info_text + n, info_size - n," %u", sharedLines[i]->ID());

  n += snprintf(info_text + n, info_size - n,"\n  mean gap: %f\n", meanGap);

  return info_text;
}


/**
 * @brief Checks if Gestalt is at position x,y
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true if feature is at this position.
 */
bool FlapAri::IsAtPosition(int x, int y)
{
  return rectangle[0]->IsAtPosition(x, y) || rectangle[1]->IsAtPosition(x, y);
}


/**
 * @brief Calculates significance from rectangles-significance and from the meanGap.
 */
void FlapAri::CalculateSignificance()
{
//	if (meanGap!=0) sig = 100*(1/meanGap);
	sig = rectangle[0]->sig + rectangle[1]->sig;
	if (meanGap!=0) sig -= meanGap*5.;
	if (sig < 0) sig = 0.;
}

}
