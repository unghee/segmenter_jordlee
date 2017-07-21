/**
 * @file Cylinder.hh
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Header of Gestalt Cylinders
 **/

#include "Cylinder.hh"
#include "Draw.hh"
#include "Line.hh"

namespace Z
{

/**
 * @brief Constructor of class Cylinder
 * @param vc Vision core
 * @param ej0 First e-junction of cylinder
 * @param ej1 Second e-junction of cylinder
 * @param g geometry significance.
 */
Cylinder::Cylinder(VisionCore *vc, EJunction *ej0, EJunction *ej1, double g)	: Gestalt(vc, CYLINDER)
{
	ejct[0] = ej0;
	ejct[1] = ej1;

	vertex[0][0] = ejct[0]->ellipse->vertex[ejct[0]->vertex];
	vertex[0][1] = ejct[0]->ellipse->vertex[Other(ejct[0]->vertex)];
	vertex[1][0] = ejct[1]->ellipse->vertex[ejct[1]->vertex];
	vertex[1][1] = ejct[1]->ellipse->vertex[Other(ejct[1]->vertex)];
	
	geometrySig = g;

//   extEllipses[0] = ee0;
//   extEllipses[1] = ee1;
// 
//   for(unsigned i=0; i<sL.Size(); i++)
//   {
// 		sharedLines.PushBack(sL[i]);
// 		sharedLinesVertex[0].PushBack(sLVtx[0][i]);
// 		sharedLinesVertex[1].PushBack(sLVtx[1][i]);
//   }
//   for(unsigned i=0; i<sharedLines.Size(); i++)						// TODO ARI: nur hier werden Cylinder zu den Linien hinzugefügt
//     Lines(sharedLines[i])->AddCylinder(id);
// 
// 	// calculate center and radius of the cone
// 	Vector2 eCenter0, eCenter1;
// 	eCenter0.x = Ellipses(ExtEllipses(extEllipses[0])->ellipse)->x;
// 	eCenter0.y = Ellipses(ExtEllipses(extEllipses[0])->ellipse)->y;
// 	eCenter1.x = Ellipses(ExtEllipses(extEllipses[1])->ellipse)->x;
// 	eCenter1.y = Ellipses(ExtEllipses(extEllipses[1])->ellipse)->y;
// 	center = (eCenter0 + eCenter1)/2.;
// 
// 	// calculate radius
// 	radius = Max(fabs((eCenter0 - center).Norm()), fabs((eCenter1 - center).Norm()));
// 	radius += Max(Ellipses(ExtEllipses(extEllipses[0])->ellipse)->b, Ellipses(ExtEllipses(extEllipses[1])->ellipse)->b);
// 
// /*printf("	eCenter: %4.2f - %4.2f - eCenter: %4.2f - %4.2f\n", eCenter0.x, eCenter0.y, eCenter1.x, eCenter1.y);
// printf("Cylinder %u: center: %4.2f - %4.2f - radius: %4.2f\n", id, center.x, center.y, radius);
// */
//   // Add Cylinder to Ellipses
//   Ellipses(ExtEllipses(ee0)->ellipse)->AddCylinder(id);
//   Ellipses(ExtEllipses(ee1)->ellipse)->AddCylinder(id);
// 

 	CalculateSignificance();
}

/**
 *	@brief Returns true, if the center of the cylinder is inside the radius of this cylinder
 *	@param cylinder Index of the other cylinder
 */
// bool Cylinder::IsInside(unsigned cylinder)
// {
//   if((Cylinders(cylinder)->center - center).Length() < radius) return true;
//   else return false; 
// }

/**
 *	@brief AddSharedLines to a existing cylinder
 *	@param sl Shared lines
 *	@param sLVtx Shared lines vertex
 */
// void Cylinder::AddSharedLines(Array<unsigned> &sL, Array<unsigned> *sLVtx) 
// {
//   for(unsigned i=0; i<sL.Size(); i++)
//   {
// 		if(!sharedLines.Contains(sL[i]))
// 		{
// 			sharedLines.PushBack(sL[i]);
// 			sharedLinesVertex[0].PushBack(sLVtx[0][i]);
// 			sharedLinesVertex[1].PushBack(sLVtx[1][i]);
// 		}
//   }
// 
// 	CalculateEqualVertex();
//  	CalculateSignificance();
// }

/**
 *	@brief Calculate, if the vertex of the cylinders are equal (outer lines do not intersect?)
 */
// void Cylinder::CalculateEqualVertex()
// {
//   unsigned equal=0, unequal=0;
// 
//   for(unsigned i=0; i<sharedLinesVertex[0].Size(); i++)
// 		if(sharedLinesVertex[0][i] == sharedLinesVertex[1][i]) equal++;
// 		else unequal++;
// 		
//   if(equal >= unequal) equalVertex = true;
//   else equalVertex = false;
// }

/**
 *	@brief Check, if geometry of cylinder shows a "right circular" cylinder
 * 	Condition:
 *	The line between ellipse1->vertex[LEFT/RIGHT] and ellipse2->vertex[LEFT/RIGHT] \n
 *	should have the same direction as the mean of the normals of the both ellipses. \n
 *		=> (meanDir ~ phi0, phi1 )
 */
// bool Cylinder::CheckGeometry()  // TODO Moved to FormCylinders
// {
// 	bool newCylinder = true;
// 
// // angle between the two ellipses (vertex to vertex) => alpha
// 	Vector2 ellDir0, ellDir1;
// 	if ((vertex[0][0] - vertex[0][1]).x != 0) ellDir0 = Normalise(vertex[0][0] - vertex[0][1]);
// 	if ((vertex[1][0] - vertex[1][1]).x != 0) ellDir1 = Normalise(vertex[1][0] - vertex[1][1]);
// 	double alpha = acos(Dot(ellDir0, ellDir1));
// 	if (alpha > M_PI/2.) alpha = M_PI - alpha;
// 
// 	// angle between the surface lines => beta
// 	Vector2 dir0, dir1;
// 	if ((vertex[0][0] - vertex[1][0]).x != 0) dir0 = Normalise(vertex[0][0] - vertex[1][0]);
// 	if ((vertex[0][1] - vertex[1][1]).x != 0) dir1 = Normalise(vertex[0][1] - vertex[1][1]);
// 	double beta = acos(Dot(dir0, dir1));
// 	if (beta > M_PI/2.) beta = M_PI - beta;
// 
// 	// angles between the surface lines and the ellipses (vertex to vertex) => gamma
// 	double gamma0 = acos(Dot(ellDir0, dir0));
// 	double gamma1 = acos(Dot(ellDir0, dir1));
// 	double gamma2 = acos(Dot(ellDir1, dir0));
// 	double gamma3 = acos(Dot(ellDir1, dir1));
// 	gamma0 = fabs(gamma0 - M_PI/2.);
// 	gamma1 = fabs(gamma1 - M_PI/2.);
// 	gamma2 = fabs(gamma2 - M_PI/2.);
// 	gamma3 = fabs(gamma3 - M_PI/2.);
// 
// 	// ratio between length of surface line and ellipse radius (max. 1:5 or 5:1)
// 	double ellDiameter0 = (vertex[0][0] - vertex[0][1]).Norm();
// 	double ellDiameter1 = (vertex[1][0] - vertex[1][1]).Norm();
// 	double length0, length1;
// 	length0 = (vertex[0][0] - vertex[1][0]).Norm();
// 	length1 = (vertex[0][1] - vertex[1][1]).Norm();
// 
// 	if (length0/ellDiameter0 < 0.2 || length0/ellDiameter0 > 5 || length1/ellDiameter1 < 0.2 || length1/ellDiameter1 > 5.)
// 		newCylinder = false;
// 
// 	
// 	/// TODO Ellipsen oben und unten sollten ungefähr gleich groß sein???
// 
// // 	printf("Öffnungswinkel Ellipsen: %4.2f - Verbindungslinien: %4.2f - Gesamt: %4.2f\n", alpha, beta, alpha*beta);
// // 	if(alpha > M_PI/4. || beta > M_PI/4.)
// // 		printf("			=> False\n");	
// // 	else
// // 		printf("			=> True\n");	
// // 	printf("			gammas %4.2f - %4.2f - %4.2f - %4.2f\n\n", gamma0, gamma1, gamma2, gamma3);
// 
// 	double th = M_PI/6.;
// 	if(alpha > th || beta > th || gamma0 > th || gamma1 > th || gamma2 > th || gamma3 > th)
// 		newCylinder = false;
// 
// 	// calculate value of geometry
// 	geometry = alpha+beta+gamma0+gamma1+gamma2+gamma3;
// 
// 	if (newCylinder) return true;
// 	else return false;
// }


/**
 *	@brief Draw cylinder
 *	@param detail Degree of detail
 */
void Cylinder::Draw(int detail)
{
	if (detail <= 2)
	{
		DrawLine2D(vertex[0][0].x, vertex[0][0].y, vertex[1][0].x, vertex[1][0].y);
		DrawLine2D(vertex[0][1].x, vertex[0][1].y, vertex[1][1].x, vertex[1][1].y);
	}
	if (detail == 0)
  {
		ejct[0]->ellipse->Draw(detail);
		ejct[1]->ellipse->Draw(detail);
	}
	if (detail >= 1)
  {
		ejct[0]->Draw(detail-1);
		ejct[1]->Draw(detail-1);
	}


//   if (detail == 0)
//   {
// 		Ellipses(ExtEllipses(extEllipses[0])->ellipse)->Draw(detail);
// 		Ellipses(ExtEllipses(extEllipses[1])->ellipse)->Draw(detail);
// 
// 		Vector2 vertex[2][2]; 		///< vertex[ELLIPSE][SIDE]
// 		if(equalVertex)
// 		{	  
// 			vertex[0][0] = ExtEllipses(extEllipses[0])->vertex[0];
// 			vertex[0][1] = ExtEllipses(extEllipses[0])->vertex[1];
// 			vertex[1][0] = ExtEllipses(extEllipses[1])->vertex[0];
// 			vertex[1][1] = ExtEllipses(extEllipses[1])->vertex[1];
// 		}
// 		else
// 		{	  
// 			vertex[0][0] = ExtEllipses(extEllipses[0])->vertex[0];
// 			vertex[0][1] = ExtEllipses(extEllipses[0])->vertex[1];
// 			vertex[1][0] = ExtEllipses(extEllipses[1])->vertex[1];
// 			vertex[1][1] = ExtEllipses(extEllipses[1])->vertex[0];
// 		}
// 			
// 		DrawLine2D(vertex[0][LEFT].x, vertex[0][LEFT].y, 
// 			vertex[1][LEFT].x, vertex[1][LEFT].y, RGBColor::red);
// 		DrawLine2D(vertex[0][RIGHT].x, vertex[0][RIGHT].y, 
// 			vertex[1][RIGHT].x, vertex[1][RIGHT].y, RGBColor::red);
//   }
// 	
//   if (detail > 0)  
//   {
//     for (unsigned i=0; i<sharedLines.Size(); i++)
// 	  Lines(sharedLines[i])->Draw(detail);
// 
// 		ExtEllipses(extEllipses[0])->Draw(detail);
//   	ExtEllipses(extEllipses[1])->Draw(detail);
//   }
}

/**
 * @brief Get Gestalt information as string.
 * @return Returns Gestalt information as string.
 */
const char* Cylinder::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s  E-Junctions: %u %u\n"
		"  Ellipses: %u %u",
		Gestalt::GetInfo(), ejct[0]->ID(), ejct[1]->ID(), ejct[0]->ellipse->ID(), ejct[1]->ellipse->ID());

// 		for (unsigned i=0; i<sharedLines.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", sharedLines[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\nvtx: ");
//   for (unsigned i=0; i<sharedLinesVertex[0].Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "(%u-%u) ", 
//   		sharedLinesVertex[0][i], sharedLinesVertex[1][i]);
//   if (equalVertex)
//   	n += snprintf(info_text + n, info_size - n, "\nequalVertex: true");
//   else
//   	n += snprintf(info_text + n, info_size - n, "\nequalVertex: false");
// 
//   n += snprintf(info_text + n, info_size - n, "\n\ncenter: %4.2f / %4.2f\nradius: %4.2f", center.x, center.y, radius);
// 
  return info_text;
}

/**
 *	@brief Returns true, if Gestalt is at this position.
 *	@param x x-coordinate
 *	@param y y-coordinate
 *	@return Returns true, if Gestalt is at this position.
 */
bool Cylinder::IsAtPosition(int x, int y)
{
	return (ejct[0]->ellipse->IsAtPosition(x, y) ||
					ejct[1]->ellipse->IsAtPosition(x, y));
}

/**
 * @brief Calculate significance.
 */
void Cylinder::CalculateSignificance()
{
	if(geometrySig!=0) sig = 100./geometrySig;
	else sig = 1000;
}

}
