/**
 * @file FormCorners.hh
 * @author Andreas Richtsfeld
 * @date September 2010
 * @version 0.1
 * @brief Gestalt class of corners.
 */

#include "Corner.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt corner.
 * @param j Junctions
 * @param l Lines
 * @param np Near points of lines
 */
Corner::Corner(VisionCore *vc, Array<LJunction*> j, Array<Line*> l, Array<unsigned> np) : Gestalt(vc, CORNER)
{
  ljcts = j;
  lines = l;
  near_points = np;
  
  // Add the new created corner to the lines
  for(unsigned i=0; i<lines.Size(); i++)
    lines[i]->corners[near_points[i]].PushBack(this);
  
//   if(lines.Size() != 3)    // TODO Is this a problem?
//     printf("Corner::Corner: Warning: Corner with more or less than 3 arms!\n");
  
// printf("NEW CORNER: ljcts: ");
// for(unsigned i=0; i<ljcts.Size(); i++)
// {
//   printf("%u  ", ljcts[i]->ID());
// }
// printf("\nlines: ");
// for(unsigned i=0; i<lines.Size(); i++)
// {
//   printf("%u@%u  ", lines[i]->ID(), near_points[i]);
// }
// printf("\n");

  CalculateProperties();
}



/*
**	TODO ARI: Umbennen (ist eher ein Update oder so)
**	Add new L-Junction to existing corner
*/
// void Corner::AddLJunctions(Array<unsigned> &lj, Array<unsigned> &li, Array<unsigned> &np)
// {
//   for (unsigned i=0; i<lj.Size(); i++)
//   {
// 	if (!ljcts.Contains(lj[i]))
// 	  ljcts.PushBack(lj[i]);
//   }
//   
//   for (unsigned i=0; i<li.Size(); i++)
//   {
// 	if(!lines.Contains(li[i]))
// 	{
// 	  lines.PushBack(li[i]);
// 	  near_point.PushBack(np[i]);
// 	  Lines(li[i])->corners.PushBack(id);
// 	}
//   }
// }


/**
 * @brief Calculate the intersection point.
 * @param gap The mean gap between the original intersection points and the new calculated intersection point
 * @param
 * @param
 */
void Corner::CalculateProperties()
{
  // get the mean between the intersection points
  Vector2 zw;
  zw.x = 0;
  zw.y = 0;
  unsigned counter = 0;
  gap = 0.;
  for(unsigned i=0; i<ljcts.Size(); i++)
  {
    zw += ljcts[i]->isct;
    counter++;
  }
  isct = zw / (double) counter;
// printf("Corner::CalculateIntersection: intersection: %4.3f - %4.3f\n", isct.x, isct.y);

  // calculate mean gap to the new intersection point!
  zw.x = 0;	zw.y = 0;
  counter = 0;
  for(unsigned i=0; i<ljcts.Size(); i++)
  {
	  Vector2 zwi = ljcts[i]->isct - isct;
// printf("Corner::CalculateIntersection: zwi: %4.3f - %4.3f\n", zwi.x, zwi.y);
// printf("Corner::CalculateIntersection: gap: %4.3f\n", Length(zwi));
	  gap += Length(zwi);
	  counter++;
  }
gap /= (double) counter;

// printf("Corner::CalculateIntersection: mean gap: %4.3f\n", gap);
  
  // calculate angles
  for(unsigned i=0; i<lines.Size(); i++)
  {
	  Vector2 zwi = lines[i]->point[Other(near_points[i])] - isct;
	  double ang = PolarAngle(zwi);
// printf("Corner::CalculateIntersection: angles: %4.0f\n", ang*180/M_PI);
	  angle.PushBack(ang);
  }
}

/*
**	Add a third L-Jct to a existing Corner
*/
// void Corner::Recalc(unsigned lj)
// {
// //  ljct[2] = lj;
// //  sig += 100;
// }


/**
 * @brief Draw Gestalt principle
 * @param detail Degree of detail
 */
void Corner::Draw(int detail)
{
  // draw intersection point
  DrawPoint2D(isct.x, isct.y);
  
  // draw arms
  if(detail == 1)
  {
    for(unsigned i=0; i<ljcts.Size(); i++)
    {
      Vector2 newP1, newP2;
      newP1.x = ljcts[i]->dir[0].x * 10;
      newP1.y = ljcts[i]->dir[0].y * 10;
      newP2.x = ljcts[i]->dir[1].x * 10;
      newP2.y = ljcts[i]->dir[1].y * 10;
      DrawLine2D(isct.x, isct.y, isct.x + newP1.x, isct.y + newP1.y, RGBColor::cyan);
      DrawLine2D(isct.x, isct.y, isct.x + newP2.x, isct.y + newP2.y, RGBColor::cyan);
    }
  }
}

/**
 * @brief Draw Gestalt principle
 * @param detail Degree of detail
 */
const char* Corner::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
  n += snprintf(info_text, info_size, "%s  L-Jcts: ", Gestalt::GetInfo());
  for (unsigned i=0; i<ljcts.Size(); i++)
	n += snprintf(info_text + n, info_size - n, "%u ", ljcts[i]->ID());
	
  n += snprintf(info_text + n, info_size - n, "\n  Lines: ");
  
  for (unsigned i=0; i<lines.Size(); i++)
	n += snprintf(info_text + n, info_size - n, "%u ", lines[i]->ID());

  n += snprintf(info_text + n, info_size - n, "\n  near Point: ");
  
  for (unsigned i=0; i<near_points.Size(); i++)
	n += snprintf(info_text + n, info_size - n, "%u ", near_points[i]);

  n += snprintf(info_text + n, info_size - n, "\n  angle: ");
  
  for (unsigned i=0; i<angle.Size(); i++)
	n += snprintf(info_text + n, info_size - n, "%4.2f ", angle[i]);

   return info_text;
  return "";
}



/**
 * @brief Draw Gestalt principle
 * @param detail Degree of detail
 */
bool Corner::IsAtPosition(int x, int y)
{
  for (unsigned i=0; i<lines.Size(); i++)
  {
    if (lines[i]->IsAtPosition(x, y))
      return true;
  }
  return false;
}


void Corner::CalculateSignificance(double gap)
{
  printf("Corner::CalculateSignificance: Not yet implemented!\n");
// //  double cGap = gap/50.;
// //  sig = 1./(cGap + 0.005);
//   sig += meanLength/(gap + 0.2);
}

}

