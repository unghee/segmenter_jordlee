/**
 * @file Cube.hh
 * @author Richtsfeld Andreas
 * @date March 2010
 * @version 0.1
 * @brief Class file of Gestalt Cube.
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"
#include "Cube.hh"
#include <cstdio>

namespace Z
{

/**
 * @brief Constructor for Gestalt Cube: Construction from three flaps.
 * @param vc Vision core
 * @param f The three flaps, which are building the cube
 */
Cube::Cube(VisionCore *vc, FlapAri *f[3]) : Gestalt(vc, CUBE)
{
	plausible = PlausibilityCheck(f);
	GetCornerPoints();
	parallelity = CheckParallelity();
	CalculateProperties();
	CalculateSignificance();
}

/**
 * @brief Check the plausability of the three flaps.
 * If flap is bent to one direction, the first rectangle is the left and the \n
 * second one the right rectangle of the flap. When we build now a cube and this \n
 * rules are fulfilled, then should f[0]->r[1] = f[1]->r[0] and so on ... (clockwise). \n
 * If this check is not fulfilled, then is the cube normaly not valid!
 * @param f The three flap references.
 */
bool Cube::PlausibilityCheck(FlapAri *f[3])
{
	// order the flaps, so that one flap[0]->rect[1] == flap[1]->rect[0] and so on.
	flap[0] = f[0];
	rectangle[0] = flap[0]->rectangle[0];
	rectangle[1] = flap[0]->rectangle[1];
	if(f[0]->rectangle[1]->ID() == f[1]->rectangle[0]->ID() || f[0]->rectangle[1]->ID() == f[2]->rectangle[0]->ID())
	{
		if(f[0]->rectangle[1]->ID() == f[1]->rectangle[0]->ID())
		{
			flap[1] = f[1];
			flap[2] = f[2];
			rectangle[2] = flap[1]->rectangle[1];
			return true;
		}
		if(f[0]->rectangle[1]->ID() == f[2]->rectangle[0]->ID())
		{
			flap[1] = f[2];
			flap[2] = f[1];
			rectangle[2] = flap[1]->rectangle[1];
			return true;
		}
	}
	else	// unplausible: we do not know how the flaps should be assigned.
	{
		flap[1] = f[1];
		flap[2] = f[2];

		if((f[0]->rectangle[0]->ID() != f[1]->rectangle[0]->ID()) && (f[0]->rectangle[1]->ID() != f[1]->rectangle[0]->ID()))
			rectangle[2] = flap[1]->rectangle[0];
		else
			rectangle[2] = flap[1]->rectangle[1];

		return false;
	}
}

/**
 * @brief Calculate the corner points from the three rectangles. We assume, that \n
 * each corner point is the mean value of three rectangle corners. \n
 * 
 */
void Cube::GetCornerPoints()
{
	cornerPoint[0] = (flap[0]->isct[0] + flap[1]->isct[0] + flap[2]->isct[0])/3.;
	cornerPoint[1] = (flap[0]->isct[1] + flap[1]->isct[5] + flap[2]->isct[3])/3.;
	cornerPoint[2] = (flap[0]->isct[2]                    + flap[2]->isct[4])/2.;
	cornerPoint[3] = (flap[0]->isct[3] + flap[1]->isct[1] + flap[2]->isct[5])/3.;
	cornerPoint[4] = (flap[0]->isct[4] + flap[1]->isct[2])                   /2.;
	cornerPoint[5] = (flap[0]->isct[5] + flap[1]->isct[3] + flap[2]->isct[1])/3.;
	cornerPoint[6] = (                   flap[1]->isct[4] + flap[2]->isct[2])/2.;

	Vector2 intersection_points[3];
	intersection_points[0] = cornerPoint[2] - (cornerPoint[0] - cornerPoint[5]);
	intersection_points[1] = cornerPoint[6] - (cornerPoint[0] - cornerPoint[3]);
	intersection_points[2] = cornerPoint[4] - (cornerPoint[0] - cornerPoint[1]);
	cornerPoint[7] = (intersection_points[0] + intersection_points[1] + intersection_points[2])/3;
}


/**
 * @brief	Calculate properties of the cube. Calculate center point, inner and outer radius.
 */
void Cube::CalculateProperties()
{
  center = (cornerPoint[0] + cornerPoint[1] + cornerPoint[2] + 
						cornerPoint[3] + cornerPoint[4] + cornerPoint[5])/6.;

	double r[5];
	for(unsigned i=0; i<=5; i++)
		r[i] = Length(cornerPoint[i] - cornerPoint[0]);
	for(unsigned i=0; i<=5; i++)
		radius = Max(r[i], r[i<6?i+1:0]);
}


/**
 *	@brief Check parallelity of the 4 cube edges in the 3 directions. Calculates always \”
 *	the deviation between the four opposing edges and adds it up.
 *	@return Returns a significance value, representing the parallelity of the cube edges.
 */
double Cube::CheckParallelity()	
{
	double parallelity = 0.;
  Vector2 edge[4];

	edge[0] = Normalise(cornerPoint[1] - cornerPoint[0]);
	edge[1] = Normalise(cornerPoint[2] - cornerPoint[3]);
	edge[2] = Normalise(cornerPoint[7] - cornerPoint[4]);
	edge[3] = Normalise(cornerPoint[6] - cornerPoint[5]);
	for(unsigned i=0; i<4; i++)
		parallelity += acos(Dot(edge[i], edge[i<3?i+1:0]));

	edge[0] = Normalise(cornerPoint[3] - cornerPoint[0]);
	edge[1] = Normalise(cornerPoint[4] - cornerPoint[5]);
	edge[2] = Normalise(cornerPoint[7] - cornerPoint[6]);
	edge[3] = Normalise(cornerPoint[2] - cornerPoint[1]);
	for(unsigned i=0; i<4; i++)
		parallelity += acos(Dot(edge[i], edge[i<3?i+1:0]));

	edge[0] = Normalise(cornerPoint[5] - cornerPoint[0]);
	edge[1] = Normalise(cornerPoint[6] - cornerPoint[1]);
	edge[2] = Normalise(cornerPoint[7] - cornerPoint[2]);
	edge[3] = Normalise(cornerPoint[4] - cornerPoint[3]);
	for(unsigned i=0; i<4; i++)
		parallelity += acos(Dot(edge[i], edge[i<3?i+1:0]));

  return parallelity;
}


/**
 * @brief Check ratio between width, depth and heigt of an cube. Ratio must be smaller than 10:1
 * This is bad!!! => Another assumption
 */
// bool Cube::CheckW2HRatio()
// {
// printf("Cube::CheckW2HRatio: Not yet implemented\n");
//
// 	bool ratio = true;
// 
// 	Vector2 meanW = ((corner_points[1][0] - corner_points[2][0]) + (corner_points[1][1] - corner_points[2][1]) +
// 									(corner_points[0][1] - corner_points[3][1]) + (corner_points[0][0] - corner_points[3][0]))/4.;
// 	Vector2 meanD = ((corner_points[3][1] - corner_points[2][1]) + (corner_points[0][1] - corner_points[1][1]) +
// 									(corner_points[0][0] - corner_points[1][0]) + (corner_points[3][0] - corner_points[2][0]))/4.;
// 	Vector2 meanH = ((corner_points[2][0] - corner_points[2][1]) + (corner_points[1][0] - corner_points[1][1]) +
// 									(corner_points[0][0] - corner_points[0][1]) + (corner_points[3][0] - corner_points[3][1]))/4.;
// 
// 	double absMeanW = meanW.Length();
// 	double absMeanD = meanD.Length();
// 	double absMeanH = meanH.Length();
// 
// 	double wdRatio, dhRatio, hwRatio;
// 	wdRatio = absMeanW/absMeanD;
// 	dhRatio = absMeanD/absMeanH;
// 	hwRatio = absMeanH/absMeanW;
// 
// 	if(wdRatio > maxRatio || wdRatio < 1./maxRatio) ratio = false;				/// ratio for "true" or "false" cubes
// 	if(dhRatio > maxRatio || dhRatio < 1./maxRatio) ratio = false;
// 	if(hwRatio > maxRatio || hwRatio < 1./maxRatio) ratio = false;
// 
// /// ARI: Warnmeldung W-D-H falsch
// if(!ratio)
// {	printf("Cube::CheckW2HRatio: mean W-D-H of cube %u: %4.2f - %4.2f - %4.2f\n", id, absMeanW, absMeanD, absMeanH);
// 	printf("	ratio: %4.2f - %4.2f - %4.2f\n", wdRatio, dhRatio, hwRatio);
// }
// 	return ratio;
// return 0.;
// }


/**
 *	@brief Check if another cube is at the same position.
 *	@param cube Cube, which could be inside this cube.
 */
bool Cube::IsInside(unsigned cube)
{
  if((Cubes(core, cube)->center - center).Length() < radius) return true;
	return false;
}

/**
 * @brief Draw Gestalt Cube.
 * @param detail Degree of detail.
 */
void Cube::Draw(int detail)
{
	if (detail == 0)
	{
		// draw first transparent lines
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y, cornerPoint[7].x, cornerPoint[7].y);
		DrawLine2D(cornerPoint[7].x, cornerPoint[7].y, cornerPoint[2].x, cornerPoint[2].y);
		DrawLine2D(cornerPoint[7].x, cornerPoint[7].y, cornerPoint[4].x, cornerPoint[4].y);

		DrawLine2D(cornerPoint[0].x, cornerPoint[0].y, cornerPoint[3].x, cornerPoint[3].y);
		DrawLine2D(cornerPoint[1].x, cornerPoint[1].y, cornerPoint[2].x, cornerPoint[2].y);
		DrawLine2D(cornerPoint[4].x, cornerPoint[4].y, cornerPoint[5].x, cornerPoint[5].y);

		DrawLine2D(cornerPoint[4].x, cornerPoint[4].y, cornerPoint[3].x, cornerPoint[3].y);
		DrawLine2D(cornerPoint[5].x, cornerPoint[5].y, cornerPoint[0].x, cornerPoint[0].y);
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y, cornerPoint[1].x, cornerPoint[1].y);

		DrawLine2D(cornerPoint[2].x, cornerPoint[2].y, cornerPoint[3].x, cornerPoint[3].y);
		DrawLine2D(cornerPoint[1].x, cornerPoint[1].y, cornerPoint[0].x, cornerPoint[0].y);
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y, cornerPoint[5].x, cornerPoint[5].y);

	}
	if (detail == 1 || detail == 2)
	{
		// draw first transparent lines
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y,
							 cornerPoint[7].x, cornerPoint[7].y, RGBColor::coral); // transparency
		DrawLine2D(cornerPoint[7].x, cornerPoint[7].y,
							 cornerPoint[2].x, cornerPoint[2].y, RGBColor::coral); // transparency
		DrawLine2D(cornerPoint[7].x, cornerPoint[7].y,
							 cornerPoint[4].x, cornerPoint[4].y, RGBColor::coral); // transparency

		DrawLine2D(cornerPoint[0].x, cornerPoint[0].y,
							 cornerPoint[3].x, cornerPoint[3].y, RGBColor::red);
		DrawLine2D(cornerPoint[1].x, cornerPoint[1].y,
							 cornerPoint[2].x, cornerPoint[2].y, RGBColor::red);
		DrawLine2D(cornerPoint[4].x, cornerPoint[4].y,
							 cornerPoint[5].x, cornerPoint[5].y, RGBColor::red);

		DrawLine2D(cornerPoint[4].x, cornerPoint[4].y,
							 cornerPoint[3].x, cornerPoint[3].y, RGBColor::red);
		DrawLine2D(cornerPoint[5].x, cornerPoint[5].y,
							 cornerPoint[0].x, cornerPoint[0].y, RGBColor::red);
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y,
							 cornerPoint[1].x, cornerPoint[1].y, RGBColor::red);

		DrawLine2D(cornerPoint[2].x, cornerPoint[2].y,
							 cornerPoint[3].x, cornerPoint[3].y, RGBColor::red);
		DrawLine2D(cornerPoint[1].x, cornerPoint[1].y,
							 cornerPoint[0].x, cornerPoint[0].y, RGBColor::red);
		DrawLine2D(cornerPoint[6].x, cornerPoint[6].y,
							 cornerPoint[5].x, cornerPoint[5].y, RGBColor::red);
	}

  if (detail == 1)
  {
		char text[20];
		snprintf(text, 20, "%u", ID());
		DrawText2D(text, cornerPoint[0].x, cornerPoint[0].y-5, RGBColor::blue);
	}

  if(detail == 2)
  {
		char text[20];
		for (unsigned i=0; i<8; i++)
		{
			snprintf(text, 20, "%u", i);
			DrawText2D(text, cornerPoint[i].x, cornerPoint[i].y-5, RGBColor::blue);
		}
	}
// 		
// 	if(detail == 2 && (closingLJct != UNDEF_ID || closingColl != UNDEF_ID))
// 	{
// 		unsigned r0 = Flaps(flap)->rects[0];
// 		unsigned r1 = Flaps(flap)->rects[1];
// 
// 		char text[20];
// 		if (closingLJct != UNDEF_ID)
// 		{
// 			snprintf(text, 20, "%u", closingLJct);
// 			DrawText2D(text, LJunctions(closingLJct)->isct.x,
// 					LJunctions(closingLJct)->isct.y, RGBColor::blue);
// 		}
// 		else
// 		{	  
// 			snprintf(text, 20, "%u", closingColl);
// 			DrawText2D(text, Collinearities(closingColl)->vertex.x,
// 					Collinearities(closingColl)->vertex.y, RGBColor::blue);
// 		}
// 	
// 	
// 		// draw the 2 rects of the flap
// 		for(unsigned i = 0; i < 4; i++)
// 		{
// 				DrawLine2D(
// 			LJunctions(Rectangles(r0)->jcts[i])->isct.x,
// 			LJunctions(Rectangles(r0)->jcts[i])->isct.y,
// 			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.x,
// 			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.y,
// 						RGBColor::red);
// 				DrawLine2D(
// 			LJunctions(Rectangles(r1)->jcts[i])->isct.x,
// 			LJunctions(Rectangles(r1)->jcts[i])->isct.y,
// 			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.x,
// 			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.y,
// 						RGBColor::red);
// 		}
// 	
// 		// draw the L-Junction with closing lines
// 		Lines(jctLines[0])->Draw(1);
// 		Lines(jctLines[1])->Draw(1);
// 		
// 		for(unsigned i=0; i<cLines[0].Size(); i++)
// 			Lines(cLines[0][i])->Draw(1);
// 		for(unsigned i=0; i<cLines[1].Size(); i++)
// 			Lines(cLines[1][i])->Draw(1);
// 		
// 		// cLines
// 		for(unsigned i=0; i<cLines[LEFT].Size(); i++)	
// 		{
// 			DrawLine2D(Lines(cLines[LEFT][i])->point[0].x, Lines(cLines[LEFT][i])->point[0].y,
// 					Lines(cLines[LEFT][i])->point[1].x,	Lines(cLines[LEFT][i])->point[1].y, RGBColor::red);
// 		}
// 		for(unsigned i=0; i<cLines[RIGHT].Size(); i++)	
// 		{
// 			DrawLine2D( Lines(cLines[RIGHT][i])->point[0].x, Lines(cLines[RIGHT][i])->point[0].y,
// 					Lines(cLines[RIGHT][i])->point[1].x, Lines(cLines[RIGHT][i])->point[1].y,  RGBColor::red);
// 		}
//   }	
//   
//   if(detail == 2 && oFlaps[0] != UNDEF_ID && oFlaps[1] != UNDEF_ID)
//   {
// 		unsigned r0 = Flaps(flap)->rects[0];
// 		unsigned r1 = Flaps(oFlaps[0])->rects[0];
// 		unsigned r2 = Flaps(oFlaps[1])->rects[0];
// 
// 		// draw the 3 rects of the flap
//     for(unsigned i = 0; i < 4; i++)
// 		{
// 			DrawLine2D(
// 			LJunctions(Rectangles(r0)->jcts[i])->isct.x,
// 			LJunctions(Rectangles(r0)->jcts[i])->isct.y,
// 			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.x,
// 			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.y,
// 						RGBColor::red);
// 				DrawLine2D(
// 			LJunctions(Rectangles(r1)->jcts[i])->isct.x,
// 			LJunctions(Rectangles(r1)->jcts[i])->isct.y,
// 			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.x,
// 			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.y,
// 						RGBColor::red);
// 			DrawLine2D(
// 			LJunctions(Rectangles(r2)->jcts[i])->isct.x,
// 			LJunctions(Rectangles(r2)->jcts[i])->isct.y,
// 			LJunctions(Rectangles(r2)->jcts[(i<3?i+1:0)])->isct.x,
// 			LJunctions(Rectangles(r2)->jcts[(i<3?i+1:0)])->isct.y,
// 						RGBColor::red);
// 		}
//   }	
// 
//   if (detail == 3)
//   {
// 		DrawEllipse2D(center.x, center.y, radius, radius , 0., RGBColor::red);
//   }

  if(detail > 3)
  {
		flap[0]->Draw(detail-4);
		flap[1]->Draw(detail-4);
		flap[2]->Draw(detail-4);
  }
}


/**
 * @brief Get information about Gestalt as text.
 * @return Returns information as text.
 */
const char* Cube::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;

	if(plausible)
	  n += snprintf(info_text, info_size, "%s  is plausible\n", Gestalt::GetInfo());
	else
	  n += snprintf(info_text + n, info_size - n, "  is not plausible\n");

  n += snprintf(info_text + n, info_size - n, "  parallelity: %4.3f (%4.1f°)\n", parallelity, parallelity*180/M_PI);

  n += snprintf(info_text + n, info_size - n, "  flaps: %u - %u - %u\n",
      flap[0]->ID(), flap[1]->ID(), flap[2]->ID());

  n += snprintf(info_text + n, info_size - n, "  rectangles: %u - %u - %u",
      rectangle[0]->ID(), rectangle[1]->ID(), rectangle[2]->ID());

  return info_text;
}


/**
 * @brief Checks, if cube is at this position.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if cube is at this position.
 */
bool Cube::IsAtPosition(int x, int y)
{
  return flap[0]->IsAtPosition(x, y) ||
				 flap[1]->IsAtPosition(x, y) ||
				 flap[2]->IsAtPosition(x, y);
}


/**
 * @brief Another method to calculate the significance for the cube.
 * Add significance from the three flaps and divide it by 2*parallelity
 */
void Cube::CalculateSignificance()
{
	sig = flap[0]->sig + flap[1]->sig + flap[2]->sig;
	if(parallelity!=0) sig = sig/(2.0*parallelity);
}

}
