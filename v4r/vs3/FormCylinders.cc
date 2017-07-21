/**
 * @file FormCylinders.cc
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Gestalt Principle FormCylinders
 **/

#include <cstdio>
#include <stdio.h>
#include "FormCylinders.hh"

namespace Z
{

static int CmpCylinders(const void *a, const void *b)
{
  if( (*(Cylinder**)a)->sig > (*(Cylinder**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormCylinders::Rank()
{
  RankGestalts(Gestalt::CYLINDER, CmpCylinders);
}

/**
 * @brief Constructor of class FormCylinders
 * @param cfg Config
 */
FormCylinders::FormCylinders(VisionCore *vc) : GestaltPrinciple(vc)
{
  needsOperate = true;
	nonIncremental = true;
}

/**
 *	@brief Mask cylinders which have the center point inside the cylinder radius of another
 *	cylinder with higher significance.
 */
void FormCylinders::Mask()																																	/// TODO TODO
{
//   for(unsigned i=0; i<NumCylinders(core); i++)
//   {
// 		for(unsigned j=0; j<NumCylinders(core); j++)
// 		{
// 			if(!Cylinders(core, i)->IsMasked() && !Cylinders(core, j)->IsMasked() && i!=j)
// 			if(Cylinders(core, i)->sig <= Cylinders(core, j)->sig)
// 			{
// 				if(Cylinders(core, i)->IsInside(j))
// 				{	
// 					Cylinders(core, i)->Mask(j);
// 				}
// 			}
// 		}
//   }

	// Mask all Cylinders, which using masked e-junctions.
	for(unsigned i=0; i<NumCylinders(core); i++)
	{
		Cylinder *cyl = (Cylinder*)core->RankedGestalts(Gestalt::CYLINDER, i);
		if(cyl->ejct[0]->IsMasked() || cyl->ejct[1]->IsMasked())
			cyl->Mask(10000);
	}
}

/**
 * @brief InformNewGestalt()	
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormCylinders::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	if(nonIncremental) return;
	StartRunTime();

  if (type == Gestalt::E_JUNCTION)
  	NewEJunction(idx);
  if (type == Gestalt::COLLINEARITY)
  	NewCollinearity(idx);

	Rank();
	Mask();

	StopRunTime();
}

/**
 * @brief Operate Gestalt principle after incremental processing.
 * This is a non-incemental implementation!
 */
void FormCylinders::PostOperate()
{	
	if(nonIncremental)
	{
		StartRunTime();
		ProcessNonIncremental();
		Mask();
		StopRunTime();
	}
}

/**
 * @brief This is a non-incemental implementation of FormCylinders.
 * TODO This is a really bad idea to compare all possibilities brute-force after 
 * incemental processing. Better would be to save all e-junctions at Gestalt ellipse and line and
 * search later for it.
 * => Combinatorial explosion.
 * TODO Find another solution => now we have all ellipse junctions available for every ellipse => does this help?
 */
void FormCylinders::ProcessNonIncremental()
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		for(unsigned j=0; j<EJunctions(core, i)->colLines.Size(); j++)
		{
			for(unsigned k=i+1; k<NumEJunctions(core); k++)
			{				
				if(EJunctions(core, i)->line == EJunctions(core, k)->line)
					CreateNewCylinder(EJunctions(core, i), EJunctions(core, k));
				for(unsigned l=0; l<EJunctions(core, k)->colLines.Size(); l++)
				{
					// the same colLines?
					if(EJunctions(core, i)->colLines[j] == EJunctions(core, k)->colLines[l])
						CreateNewCylinder(EJunctions(core, i), EJunctions(core, k));
					else if(EJunctions(core, i)->colLines[j] == EJunctions(core, k)->line)
						CreateNewCylinder(EJunctions(core, i), EJunctions(core, k));
					else if(EJunctions(core, i)->line == EJunctions(core, k)->colLines[l])
						CreateNewCylinder(EJunctions(core, i), EJunctions(core, k));
				}
			}
		}
	}
}

/**
 * @brief Create a new cylinder, when there is:
 * - at least one e-junction at the other side of the ellipse.
 */
void FormCylinders::CreateNewCylinder(EJunction *ej0, EJunction *ej1)
{
	bool isEllipse = true;
	if(ej0->ellipse->ID() == ej1->ellipse->ID()) isEllipse = false;
	
	if(ej0->ellipse->ejcts[0].Size() == 0 && ej0->ellipse->ejcts[1].Size() == 0 && 
		 ej1->ellipse->ejcts[0].Size() == 0 && ej1->ellipse->ejcts[1].Size() == 0) isEllipse = false;
	
	double geometry;
	if(!CheckGeometry(ej0, ej1, geometry)) isEllipse = false;
	
	if(!IsCylinder(ej0->ellipse, ej1->ellipse) && isEllipse)
			core->NewGestalt(GestaltPrinciple::FORM_CYLINDERS, new Cylinder(core, ej0, ej1, geometry));
}


/**
 *	@brief Check, if geometry of cylinder shows a "right circular" cylinder
 * 	Condition:
 *	The line between ellipse1->vertex[LEFT/RIGHT] and ellipse2->vertex[LEFT/RIGHT] \n
 *	should have the same direction as the mean of the normals of the both ellipses. \n
 *		=> (meanDir ~ phi0, phi1 )
 */
bool FormCylinders::CheckGeometry(EJunction *ej0, EJunction *ej1, double &geometry)
{
	Vector2 vertex[2][2];
	vertex[0][0] = ej0->ellipse->vertex[ej0->vertex];
	vertex[0][1] = ej0->ellipse->vertex[Other(ej0->vertex)];
	vertex[1][0] = ej1->ellipse->vertex[ej1->vertex];
	vertex[1][1] = ej1->ellipse->vertex[Other(ej1->vertex)];

	bool newCylinder = true;

// angle between the two ellipses (vertex to vertex) => alpha
	Vector2 ellDir0, ellDir1;
	if ((vertex[0][0] - vertex[0][1]).x != 0) ellDir0 = Normalise(vertex[0][0] - vertex[0][1]);
	if ((vertex[1][0] - vertex[1][1]).x != 0) ellDir1 = Normalise(vertex[1][0] - vertex[1][1]);
	double alpha = acos(Dot(ellDir0, ellDir1));
	if (alpha > M_PI/2.) alpha = M_PI - alpha;

	// angle between the surface lines => beta
	Vector2 dir0, dir1;
	if ((vertex[0][0] - vertex[1][0]).x != 0) dir0 = Normalise(vertex[0][0] - vertex[1][0]);
	if ((vertex[0][1] - vertex[1][1]).x != 0) dir1 = Normalise(vertex[0][1] - vertex[1][1]);
	double beta = acos(Dot(dir0, dir1));
	if (beta > M_PI/2.) beta = M_PI - beta;

	// angles between the surface lines and the ellipses (vertex to vertex) => gamma
	double gamma0 = acos(Dot(ellDir0, dir0));
	double gamma1 = acos(Dot(ellDir0, dir1));
	double gamma2 = acos(Dot(ellDir1, dir0));
	double gamma3 = acos(Dot(ellDir1, dir1));
	gamma0 = fabs(gamma0 - M_PI/2.);
	gamma1 = fabs(gamma1 - M_PI/2.);
	gamma2 = fabs(gamma2 - M_PI/2.);
	gamma3 = fabs(gamma3 - M_PI/2.);

	// ratio between length of surface line and ellipse radius (max. 1:5 or 5:1)
	double ellDiameter0 = (vertex[0][0] - vertex[0][1]).Norm();
	double ellDiameter1 = (vertex[1][0] - vertex[1][1]).Norm();
	double length0, length1;
	length0 = (vertex[0][0] - vertex[1][0]).Norm();
	length1 = (vertex[0][1] - vertex[1][1]).Norm();

	if (length0/ellDiameter0 < 0.2 || length0/ellDiameter0 > 5 || length1/ellDiameter1 < 0.2 || length1/ellDiameter1 > 5.)
		newCylinder = false;

	
	/// TODO Ellipsen oben und unten sollten ungefähr gleich groß sein???
// 	printf("Öffnungswinkel Ellipsen: %4.2f - Verbindungslinien: %4.2f - Gesamt: %4.2f\n", alpha, beta, alpha*beta);
// 	if(alpha > M_PI/4. || beta > M_PI/4.)
// 		printf("			=> False\n");	
// 	else
// 		printf("			=> True\n");	
// 	printf("			gammas %4.2f - %4.2f - %4.2f - %4.2f\n\n", gamma0, gamma1, gamma2, gamma3);

	double th = M_PI/6.;
	if(alpha > th || beta > th || gamma0 > th || gamma1 > th || gamma2 > th || gamma3 > th)
		newCylinder = false;

	// calculate value of geometry
	geometry = alpha+beta+gamma0+gamma1+gamma2+gamma3;

	if (newCylinder) return true;
	else return false;
}


// ------------------------------------------------------------- //
// --------------------- Incremental Stuff --------------------- //
// ------------------------------------------------------------- //

/**
 *	@brief NewEJunction(): Try to create a new cylinder, if a new E-Junction is available
 *	@param ejct E-Junction index
 */
void FormCylinders::NewEJunction(unsigned ejct)
{
	// get ellipse and lines
	Ellipse *ell0 = EJunctions(core, ejct)->ellipse;
	Line *line = EJunctions(core, ejct)->line;
	unsigned lineEnd = EJunctions(core, ejct)->lineEnd;
	
	/// for the first line: is there another e-junction with that line? => create new cylinder
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		if(EJunctions(core, i)->IsLine(line) && i != ejct)
		{
			Ellipse *ell1 = Ellipses(core, EJunctions(core, i)->ellipse->ID());
			if(!IsCylinder(ell0, ell1))
			{
				double geometry = 0.; // TODO Check geometry
				core->NewGestalt(GestaltPrinciple::FORM_CYLINDERS, new Cylinder(core, EJunctions(core, ejct), EJunctions(core, i), geometry));
			}
				/// TODO Estimate vertices of both ellipses.
				/// TODO Estimate the whole combination of lines?
				/// TODO 
		}
	}
	
	// follow line, search for second ellipse (junction).
	GetNext(EJunctions(core, ejct), line, lineEnd);
}

/**
 * @brief Find the next following lines (via collinearities) and search for ellipse-junction. \n
 * Create new Gestalt Cylinder, if ellipse is at the other end.
 * @param ell0 Origin ellipse for cylinder
 * @param line Next connected line to find other ellipse junction
 * @param lineEnd End of the line, we should search
 */
void FormCylinders::GetNext(EJunction *ejct, Line *line, unsigned lineEnd)
{
	for(unsigned i=0; i<line->coll[lineEnd].Size(); i++)
	{
		Collinearity *c = line->coll[lineEnd][i];
		Line *l = c->line[Other(c->WhichLineIs(line))];
		unsigned le = c->WhichEndIs(l);
		
		// is there an e-junction with this coll-lines and it is not the same e-junction
		for(unsigned j=0; j<NumEJunctions(core); j++)
		{
			if(EJunctions(core, j)->IsLine(l) && j != ejct->ID())
			{
				Ellipse *ell1 = Ellipses(core, EJunctions(core, j)->ellipse->ID());
				if(!IsCylinder(ejct->ellipse, ell1))
				{
					double geometry = 0.; // TODO Check geometry
					core->NewGestalt(GestaltPrinciple::FORM_CYLINDERS, new Cylinder(core, ejct, EJunctions(core, j), geometry));
				}
					/// TODO We need the ellipse-junctions!!! (instead of ellipses)
					/// TODO Estimate vertices of both ellipses.
					/// TODO Estimate the whole combination of lines?
					/// TODO 
			}
		}
		
		// follow line, search for another ellipse
		GetNext(ejct, l, Other(le));
	}
}

/**
 *	@brief Try to create a new cylinder, if a new Collinearity is available.
 *	@param coll ID of new collinearity
 */
void FormCylinders::NewCollinearity(unsigned coll)
{
	printf("FormCylinders::NewCollinearity: Not yet implemented!\n");
}

/**
 *	@brief NewCylinder(): Make new cylinder, if cylinder don´t already exists
 *	Other conditions ???
 *	@param eE0 Extended ellipse 0
 *	@param eE1 Extended ellipse 1
 *	@param sL Shared lines
 *	@param sLVtx Shared lines vertices
 */
void FormCylinders::NewCylinder(unsigned eE0, unsigned eE1, Array<unsigned> &sL, Array<unsigned> *sLVtx)
{
//   // Check if cylinder already exists
//   bool newCylinder = true;
//   Array<unsigned> cyl = Ellipses(ExtEllipses(eE0)->ellipse)->cylinders;
// 	
//   for(unsigned i=0; i<cyl.Size(); i++)
//   {
// 		if((Cylinders(cyl[i])->extEllipses[0] == eE0 &&	Cylinders(cyl[i])->extEllipses[1] == eE1) ||
// 			(Cylinders(cyl[i])->extEllipses[0] == eE1 && Cylinders(cyl[i])->extEllipses[1] == eE0))
// 		{
// 			Cylinders(cyl[i])->AddSharedLines(sL, sLVtx);
// 			newCylinder = false;
// 		}
//   }
// 
//   if (newCylinder)
//   {
// /*		// Condition:
// 		// The line between ellipse1->vertex[LEFT/RIGHT] and 
// 		// ellipse2->vertex[LEFT/RIGHT] should have the same direction as the mean
// 		// of the normals of the both ellipses. => (meanDir ~ phi0, phi1 )
// 	
// 		Vector2 vertex[2][2]; 		///< vertex [ELL0/ELL1] [LEFT/RIGHT]
// 		vertex[0][0] = ExtEllipses(eE0)->vertex[0];
// 		vertex[0][1] = ExtEllipses(eE0)->vertex[1];
// 		vertex[1][0] = ExtEllipses(eE1)->vertex[0];
// 		vertex[1][1] = ExtEllipses(eE1)->vertex[1];
// 		
// 		// mean direction: Normals of the ellipses 
// 		double phiEll0 = Ellipses(ExtEllipses(eE0)->ellipse)->phi;
// 		double phiEll1 = Ellipses(ExtEllipses(eE1)->ellipse)->phi;
// 		double meanDir = ScaleAngle_0_pi(M_PI/2. + (phiEll0 + phiEll1)/2.);			// stimmt das so???
// 	
// 		// calculate direction of the 2 connecting "lines" 
// 		double diff0 = 0, diff1 = 0, phi0 = 0, phi1 = 0;
// 		
// 		double x0 = vertex[0][0].x - vertex[1][0].x;
// 		double y0 = vertex[0][0].y - vertex[1][0].y;
// 		double x1 = vertex[0][1].x - vertex[1][1].x;
// 		double y1 = vertex[0][1].y - vertex[1][1].y;
// 	
// 		if (x0 != 0 && x1 != 0)
// 		{
// 			phi0 = atan(y0/x0);	// angle of connecting "lines"
// 			phi1 = atan(y1/x1);
// 			if (phi0 < 0) phi0 += M_PI;
// 			if (phi1 < 0) phi1 += M_PI;
// 				
// 			// deviation between ellipse meanDir and angle of connecting "lines"
// 			diff0 = fabs(meanDir - ScaleAngle_0_pi(phi0)); 
// 			diff1 = fabs(meanDir - ScaleAngle_0_pi(phi1)); 
// 		}
// 		else printf("Error: FormCylinders: x0 or x1 is 0!\n");
// 				
// 		double threshold = M_PI/4.;		// Threshold for the "same" direction
// 		
// 		// Deviation between connecting lines and the mean ellipse normals
// 		if (diff0 < threshold && diff1 < threshold)
// 			newCylinder = true;
// 		else
// 			newCylinder = false;
// 
// 		// The two connecting lines should be "parallel"
// 		// The ellipse normals should be the "same"
//     if (fabs(phi0 - phi1) > threshold || fabs(phiEll0-phiEll1) > threshold) newCylinder = false;
// */
// // TODO ARI: Aussortieren der Cylinder ansehen!
// //printf("\nCylinder %i: ", NumCylinders());
// //printf(" phiEll0, phiEll1, meanDir: %f, %f, %f\n", phiEll0, phiEll1, meanDir);
// //printf("fabs(phiEll0-phiEll1): %f\n", fabs(phiEll0-phiEll1));
// 
// /*    if (fabs(phi0 - phi1) > threshold) printf("		FALSE: %f\n", fabs(phi0 - phi1));
//       if(newCylinder) printf("true\n");
// 	else printf("false\n");
// 		
// 	printf(" diff0, diff1: %f - %f\n", diff0, diff1);  
// 	printf(" phiEll0, phiEll1, meanDir: %f, %f, %f\n", phiEll0, phiEll1, meanDir);
// */
// 		if (newCylinder) NewGestalt(new Cylinder(eE0, eE1, sL, sLVtx));
//   }  
}

bool FormCylinders::NeedsOperate()
{ 
  return needsOperate;	
}

/**
 * @brief Checks if the cylinder with the two ellipses already exists.
 * TODO TODO Another possibility would be to mask the bad results later!
 */
bool FormCylinders::IsCylinder(Ellipse *e0,  Ellipse *e1)
{
	for(unsigned i=0; i<NumCylinders(core); i++)
		if((Cylinders(core, i)->ejct[0]->ellipse == e0 && Cylinders(core, i)->ejct[1]->ellipse == e1) ||
			 (Cylinders(core, i)->ejct[1]->ellipse == e0 && Cylinders(core, i)->ejct[0]->ellipse == e1)) return true;
	return false;
}

}





