/**
 * @file FormCones.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Implementation of Gestalt-principle FormCones
 **/

#include "FormCones.hh"
#include "Cone.hh"
#include "Ellipse.hh"

namespace Z
{

static int CmpCones(const void *a, const void *b)
{
  if( (*(Cone**)a)->sig > (*(Cone**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormCones::Rank()
{
  RankGestalts(Gestalt::CONE, CmpCones);
}

/**
 *	@brief Masking bad results of spheres
 */
void FormCones::Mask()											// TODO masking!
{
//   for(unsigned i=0; i<NumCones(core); i++)
//   {
// 		for(unsigned j=0; j<NumCones(core); j++)
// 		{
// 			if(!Cones(core, i)->IsMasked() && !Cones(core, j)->IsMasked())
// 			if(Cones(core, i)->sig < Cones(core, j)->sig)
// 			{
// 				if(Cones(core, i)->IsInside(Cone(core, j)))
// 				{	
// 					Cones(core, i)->Mask(j);		 
// 					printf("Mask cone: %u\n", cone(core, i)->ID());
// 				}
// 			}		  
// 		}
//   }

	// Mask all cones, which using a masked ellipse.
	for(unsigned i=0; i<NumCones(core); i++)
	{
		Cone *cone = (Cone*)core->RankedGestalts(Gestalt::CONE, i);
		if(cone->ejct[0]->ellipse->IsMasked())
			cone->Mask(10000);
	}

}

bool FormCones::NeedsOperate()
{ 
  return false;	
}


/**
 * @brief Constructor of Gestalt-principle FormSpheres: Creates the Gestalt Sphere from
 * the underlying Gestalts ellipses.
 */
FormCones::FormCones(VisionCore *vc) : GestaltPrinciple(vc)
{

}

/**
 * @brief Inform principle about new Gestalt.
 * @param type Type of Gestalt
 * @param idx Index of Gestalt
 * TODO The implementation of NewEJunction should also work for this incremental
 * function. When we want to use it incremental, than also implement NewLJunction and NewCollinearity.
 */
void FormCones::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();

  switch(type)
  {
//     case Gestalt::E_JUNCTION:
// 	  NewEJunction(idx);
// 	  break;
//     case Gestalt::L_JUNCTION:
// 	  NewLJunction(idx);
//    case Gestalt::COLLINEARITY:				/// TODO Wenn neue Collinearity gefunden wird, könnte es ein E-junction update der col-lines geben!
// 	  NewCollinearity(idx);
      break;
    default:
      break;
  }
  
  StopRunTime();
}


/**
 * @brief Preprocessing (or non-incremental processing) of Gestalt principle.
 */
void FormCones::PostOperate()
{
	StartRunTime();
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		NewEJunction(i);
	}
	for(unsigned i=0; i<NumCollinearities(core); i++)
	{
		NewLJunction(i);
	}
	Rank();
	Mask();
	StopRunTime();
}


/**
 *	@brief Try to create a new cone, when a new e-junction appears.
 *	Find all other e-junction with the same ellipse (other vertex) and find a "way" from 
 *	one vertex to the other via one L-junction
 *	TODO Vom Aufruf von PostOperate aus werden hier alle Möglichen e-junctions doppelt verarbeitet: z.B. 109-41 und 41-109!!!
 *	@param ejct Index of new E-Junction
 */
void FormCones::NewEJunction(unsigned ejct)
{
	EJunction *ej0 = EJunctions(core, ejct);
	// get all e-junctions with the same ellipse.
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		if((EJunctions(core, i)->ellipse == ej0->ellipse) && 
			 (EJunctions(core, i)->vertex != ej0->vertex))
		{
			EJunction *ej0 = EJunctions(core, ejct);
			EJunction *ej1 = EJunctions(core, i);
			
			for(unsigned l=0; l<2; l++)	// LEFT/RIGHT ljcts
			{
				for(unsigned k=0; k<ej0->line->l_jct[Other(ej0->lineEnd)][l].Size(); k++)	/// line-line
				{
					LJunction *lj = ej0->line->l_jct[Other(ej0->lineEnd)][l][k];
					bool rightLineEnd = true;																		/// TODO TODO TODO am richtigen Linienende?
					if((lj->line[0] == ej1->line || lj->line[1] == ej1->line) && rightLineEnd)
					{
// 						printf("line-line\n");
						NewCone(ej0, ej1, lj);
					}
				}
				for(unsigned k=0; k<ej0->line->l_jct[Other(ej0->lineEnd)][l].Size(); k++)	/// line-colLines
				{
					LJunction *lj = ej0->line->l_jct[Other(ej0->lineEnd)][l][k];
					bool rightLineEnd = true;																		/// TODO TODO TODO am richtigen Linienende?
					for(unsigned j=0; j<ej1->colLines.Size(); j++)
					{
						if((lj->line[0] == ej1->colLines[j] || lj->line[1] == ej1->colLines[j]) && rightLineEnd)
						{
// 							printf("line-colLine\n");
							NewCone(ej0, ej1, lj);
						}
					}
				}
				for(unsigned j=0; j<ej0->colLines.Size(); j++)														/// colLines-line
				{
					for(unsigned k=0; k<ej0->colLines[j]->l_jct[Other(ej0->colLinesEnd[j])][l].Size(); k++)
					{
						LJunction *lj = ej0->colLines[j]->l_jct[Other(ej0->colLinesEnd[j])][l][k];
						if(lj->line[0] == ej1->line || lj->line[1] == ej1->line)
						{
// 							printf("colLine-line\n");
							NewCone(ej0, ej1, lj);
						}
					}
				}
				for(unsigned j=0; j<ej0->colLines.Size(); j++)														/// colLines-colLines
				{
					for(unsigned k=0; k<ej0->colLines[j]->l_jct[Other(ej0->colLinesEnd[j])][l].Size(); k++)
					{
						LJunction *lj = ej0->colLines[j]->l_jct[Other(ej0->colLinesEnd[j])][l][k];
						
						for(unsigned l=0; l<ej1->colLines.Size(); l++)
						{
							if(lj->line[0] == ej1->colLines[l] || lj->line[1] == ej1->colLines[l])
							{
// 								printf("colLine-colLine\n");
								NewCone(ej0, ej1, lj);
							}
						}
					}
				}
			}
		}
	}
}


/**
 *	@brief NewLJunction()
 *	@param ljct Index of new ljct
 */
void FormCones::NewLJunction(unsigned ljct)
{
//   // get the extEllipses from both lines of the L-Junction
//   Array<unsigned> eE0 = Lines(LJunctions(ljct)->line[0])->extEllipses;
//   Array<unsigned> eE1 = Lines(LJunctions(ljct)->line[1])->extEllipses;
// 	
//   // compare all extEllipses of both lines
//   for(unsigned i=0; i<eE0.Size(); i++)
//   {
// 		if(eE1.Contains(eE0[i]))
// 		{
// 			unsigned ellipse = ExtEllipses(eE0[i])->ellipse;
// 			unsigned eL0 = LJunctions(ljct)->line[0];
// 			unsigned eL1 = LJunctions(ljct)->line[1];
// 	
// 			// lines: l-junction and e-junctions end/vertex
// 			unsigned eL0LJctEnd = LJunctions(ljct)->near_point[0];	// LJct-End
// 			unsigned eL1LJctEnd = LJunctions(ljct)->near_point[1];  
// 			unsigned eL0EJctEnd;		// EJct-End?
// 			unsigned eL1EJctEnd;
// 			unsigned eL0EJctVertex;	// EJct-Vertex?
// 			unsigned eL1EJctVertex;				
// 			
// 			for(unsigned j=0; j<ExtEllipses(eE0[i])->extLines.Size(); j++)
// 			{
// 				if (eL0 == ExtEllipses(eE0[i])->extLines[j])
// 				{
// 					eL0EJctEnd = ExtEllipses(eE0[i])->extLinesEnd[j];
// 					eL0EJctVertex = ExtEllipses(eE0[i])->extLinesVertex[j];
// 				}
// 				if (eL1 == ExtEllipses(eE0[i])->extLines[j])
// 				{
// 					eL1EJctEnd = ExtEllipses(eE0[i])->extLinesEnd[j];
// 					eL1EJctVertex = ExtEllipses(eE0[i])->extLinesVertex[j];
// 				}
// 			}
// 			for(unsigned j=0; j<ExtEllipses(eE0[i])->colLines.Size(); j++)
// 			{
// 				if (eL0 == ExtEllipses(eE0[i])->colLines[j])
// 				{
// 					eL0EJctEnd = ExtEllipses(eE0[i])->colLinesEnd[j];
// 					eL0EJctVertex = ExtEllipses(eE0[i])->colLinesVertex[j];
// 				}
// 				if (eL1 == ExtEllipses(eE0[i])->colLines[j])
// 				{
// 					eL1EJctEnd = ExtEllipses(eE0[i])->colLinesEnd[j];
// 					eL1EJctVertex = ExtEllipses(eE0[i])->colLinesVertex[j];
// 				}
// 			}
// 		
// 			// e-jct and l-jct at different line-ends
// 			// lines on different vertex of ellipse
// 			if (eL0LJctEnd != eL0EJctEnd &&	eL1LJctEnd != eL1EJctEnd &&	eL0EJctVertex != eL1EJctVertex)
// 				NewCone(ellipse, eL0, eL1, ljct);
// 		}
//   }
}

/**
 *	@brief Generate new Gestalt cone.
 *	@param ell Ellipse index
 *	@param eL0 First ellipse line
 *	@param eL1 Second ellipse line
 *	@param ljct Index of L-Junction
 *	@TODO Überprüfungen einbauen?
 */
void FormCones::NewCone(EJunction *ej0, EJunction *ej1, LJunction *l)
{
  // Check if cone already exists
  for(unsigned i=0; i<NumCones(core); i++)
		if((Cones(core, i)->ejct[0] == ej0 && Cones(core, i)->ejct[1] == ej1) ||
			 (Cones(core, i)->ejct[1] == ej0 && Cones(core, i)->ejct[0] == ej1))
			return;
	
	// Check geometry of cone and create it, if geometry standards are fullfilled.
	double geom;
	if(CheckGeometry(ej0, ej1, l, geom))
		core->NewGestalt(GestaltPrinciple::FORM_CONES, new Cone(core, ej0, ej1, l, geom));
// 	else printf("CheckGeometry false!\n");
}


/**
 * @brief Check geometry of cone. \n
 * When cone does not fit into searched geometry standards, return false: \n
 * - When ellipse of cone is too small (<10px)
 * - When top (ljct) of the cone is inside the ellipse.
 * - When cone is not rectangular (cone is not equilateral)
 * @param ej0 First e-junction
 * @param ej1 Second e-junction
 * @param l L-junction
 * @param diff Difference of cone limb angles, representing cone significance \n
 * (perpendicular) to the ellipse.
 * @return False, if cone does not fit into geometry standards.
 */
bool FormCones::CheckGeometry(EJunction *ej0, EJunction *ej1, LJunction *l, double &diff)
{
	// Do not create a cone, if ellipse radius a is smaller than 10 pixel.
	if(ej0->ellipse->a < 10.) return false;

	// return false, if L-Junction is inside the ellRadius (cone from top)
	Vector2 ellCenter;
	ellCenter.x = ej0->ellipse->x;
	ellCenter.y = ej0->ellipse->y;
	double ellRadius = (ellCenter - ej0->ellipse->vertex[0]).Length();
	if((ellCenter - l->isct).Length() < ellRadius) return false;

	// cone is a right circular cone? => calculate the angles of left and right limb to the ellipse normal.
	Vector2 dirEll = Normalise(ej0->ellipse->vertex[0] - ej0->ellipse->vertex[1]);
	dirEll = Rotate(dirEll, M_PI/2);
	Vector2 dirLeft = Normalise(l->isct - ej0->ellipse->vertex[0]);
	Vector2 dirRight = Normalise(l->isct - ej0->ellipse->vertex[1]);

	// if opening angle of both lines goes to M_PI/2, then the difference will be smaller and smaller.
	double oAngle0 = acos(Dot(dirEll, dirLeft));	
	double oAngle1 = acos(Dot(dirEll, dirRight));
	if (oAngle0 > M_PI/2.) oAngle0 = M_PI - oAngle0;
	if (oAngle1 > M_PI/2.) oAngle1 = M_PI - oAngle1;

	diff = fabs(acos(Dot(dirEll, dirLeft)) - acos(Dot(dirEll, dirRight)));
	if (diff > 0.3 || oAngle0 > 1.3 || oAngle1 > 1.3) return false;				/// TODO Thresholds for right circular cone

// printf("	phis: %4.2f - %4.2f - %4.2f\n", PolarAngle(dirEll), PolarAngle(dirLeft), PolarAngle(dirRight));
// printf("	Diff: %4.2f\n", diff);
// printf("	oAngles: %4.2f - %4.2f\n", oAngle0, oAngle1);

/*	printf("Cone.cc - cone: %u - left: %4.2f - right: %4.2f - diff: %4.3f\n", id, acos(Dot(dirEll, dirLeft)), acos(Dot(dirEll, dirRight)), diff);
	printf("oAngle0: %4.2f - oAngle1: %4.2f\n", oAngle0, oAngle1);*/
 
 	return true; 
}


}
