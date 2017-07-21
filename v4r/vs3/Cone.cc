/**
 * @file Cone.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt Cone
 **/

#include "Draw.hh"
#include "Cone.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Constructor of class Sphere.
 *	@param vc Vision core
 *	@param e0 First ellipse junction.
 *	@param e1 Second ellipse junction.
 *	@param l L-junction
 *	@param gs Significance value of geometry.
 *	@param rad Radius of the estimated sphere.
 *	@param rat Ratio between the axis of the ellipse.
 */
Cone::Cone(VisionCore *vc, EJunction *e0, EJunction *e1, LJunction *l, double gS) : Gestalt(vc, CONE)
{
	ejct[0] = e0;
	ejct[1] = e1;
	ljct = l;
	geoSig = gS;
	
	CalculateSignificance();
}

/**
 * @brief Checks, if another cone is inside of the area of this cone.
 * Compare the distance between the center points with the ellipse radius a.
 * @param Cone Cone, to be checked with this cone.
 * @return Return true, if cone c is at the same position as this cone.
 */
bool Cone::IsInside(Cone *c)
{
	// Abstand center zu c->center muss größer sein als this->ellipse->a
	if((c->ejct[0]->ellipse->center - ejct[0]->ellipse->center).Length() < ejct[0]->ellipse->a) return true;
	else return false;
}

	
/**
 *	@brief Draw the Gestalt in the MainWin
 *	@param detail Degree of details to draw
 */
void Cone::Draw(int detail)
{
	if(detail == 0)
	{
		DrawEllipse2D(ejct[0]->ellipse->x, ejct[0]->ellipse->y, ejct[0]->ellipse->a, ejct[0]->ellipse->b, ejct[0]->ellipse->phi);
		DrawLine2D(ljct->isct.x, ljct->isct.y, ejct[0]->ellipse->vertex[LEFT].x, ejct[0]->ellipse->vertex[LEFT].y);
		DrawLine2D(ljct->isct.x, ljct->isct.y, ejct[0]->ellipse->vertex[RIGHT].x, ejct[0]->ellipse->vertex[RIGHT].y);
	}
	else if(detail ==1)
	{
		ejct[0]->ellipse->Draw(0);
		ljct->Draw(1);
	}
}

/**
 *	@brief Get information for displaying at InfoDrawArea
 */
const char* Cone::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s  ejcts: %u - %u\n  ellipse: %u\n  ljct: %u", Gestalt::GetInfo(),
								ejct[0]->ID(), ejct[1]->ID(), ejct[0]->ellipse->ID(), ljct->ID());

	return info_text;
}

/**
 *	@brief Returns true if the Gestalt is at this position.
 *	@param x x-Coordinate
 *	@param y y-Coordinate
 *	@return Returns true, if the Gestalt is at this position.
 */
bool Cone::IsAtPosition(int x, int y)
{
	if(ejct[0]->IsAtPosition(x, y)) return true;
  return false;
}

/**
 *	@brief Calcultes the significance for the Gestalt Ball
 *	@param r Ratio between the length of the axis from the ellipse
 */
void Cone::CalculateSignificance()
{
	sig = ejct[0]->ellipse->sig * geoSig;
}

}
