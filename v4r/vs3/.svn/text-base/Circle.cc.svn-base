/**
 * @file Circle.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt Circle
 **/

#include "Draw.hh"
#include "Circle.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Constructor of class Circle.
 *	@param e Ellipse
 *	@param rad Radius of the estimated sphere.
 *	@param rat Ratio between the axis of the ellipse.
 */
Circle::Circle(VisionCore *vc, Ellipse *e, double rad, double rat) : Gestalt(vc, CIRCLE)
{
	ellipse = e;
	radius = rad;

	CalculateSignificance(rat);
}

/**
 *	@brief Draw the Gestalt in the MainWin
 *	@param detail Degree of details to draw
 */
void Circle::Draw(int detail)
{
	DrawEllipse2D(ellipse->x, ellipse->y, ellipse->a, ellipse->b, ellipse->phi);
}

/**
 *	@brief Get information for displaying at InfoDrawArea
 */
const char* Circle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s  ellipse: %u\n  radius: %4.1f\n  roundness: %4.2f", Gestalt::GetInfo(), ellipse->ID(), radius, roundness);

	return info_text;
}

/**
 *	@brief Returns true if the Gestalt is at this position.
 *	@param x x-Coordinate
 *	@param y y-Coordinate
 *	@return Returns true, if the Gestalt is at this position.
 */
bool Circle::IsAtPosition(int x, int y)
{
 	if(ellipse->IsAtPosition(x, y)) return true;
  return false;
}

/**
 *	@brief Calcultes the significance for the Gestalt Circle
 *	@param r Ratio between the length of the axis from the ellipse
 */
void Circle::CalculateSignificance(double r)
{
	double f = fabs(1-r);
	sig = ellipse->sig/(f+1);	// TODO noch genau überlegen
}

}
