/**
 * @file FormEJunctions.cc
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt principle FormEJunction (ellipse junction).
 **/

#include <cstdio>
#include "FormEJunctions.hh"

namespace Z
{

/**
 * @brief Constructor of class FormEJunctions.
 * @param vc Vision core
 */
FormEJunctions::FormEJunctions(VisionCore *vc) : GestaltPrinciple(vc)
{
	initialized = false;
	typ.Clear();
	id.Clear();
}

/**
 * @brief Destructor of class FormEJunctions.
 */
FormEJunctions::~FormEJunctions()
{}

/**
 * @brief Reset the principle. Clear vote image.
 */
void FormEJunctions::Reset()
{
	initialized = false;
	typ.Clear();
	id.Clear();
}

/**
 * @brief Inform the Gestalt principle about new Gestalts.
 * When a new Ellipse comes up, we have to extend the vote image search line space.
 * We get the first ConvexArcGroups before vote image is initialized. Store the Gestalts
 * and initialize them, when the vote image is ready.
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormEJunctions::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
// printf("FormEJunctions::InformNewGestalt: start!\n");

	if(!initialized && !core->VI()->IsInitialized())		// save type and idx
	{
		typ.PushBack(type);
		id.PushBack(idx);
	}
	else
	{
		if(!initialized) Initialize();

		Array<unsigned> slines;
		Array<VoteImage::Elem> iscts;

		if(type == Gestalt::ELLIPSE)
		{
			if(core->VI()->InitEllipseSearchLines((Ellipse*)core->Gestalts(Gestalt::ELLIPSE, idx), slines, iscts)!= 0)
			{
// printf("FormEJunctions::InformNewGestalt: init ellipse during inc. proc.: junction found: %u\n", iscts.Size());
				for(unsigned j=0; j<iscts.Size(); j++)
					CreateJunction(slines[j], iscts[j]);
			}
		}
		
		if(type == Gestalt::COLLINEARITY)
		{
			UpdateEJunctions(idx);
		}
	}
	
// printf("FormEJunctions::InformNewGestalt: end!\n");
	StopRunTime();
}


/**
 * @brief Initialize the former stored ellipses.
 */
void FormEJunctions::Initialize()
{
	if(initialized) return;
	Array<unsigned> slines;
	Array<VoteImage::Elem> iscts;
	for(unsigned i=0; i<typ.Size(); i++)
		if(core->VI()->InitEllipseSearchLines((Ellipse*)core->Gestalts(typ[i], id[i]), slines, iscts)!= 0)
			for(unsigned j=0; j<iscts.Size(); j++)
				CreateJunction(slines[j], iscts[j]);
	initialized = true;
}

/**
 * @brief Post operate the Gestalt principle.
 * Mask the e-junctions after processing.
 */
void FormEJunctions::PostOperate()
{
	StartRunTime();
	Mask();
	StopRunTime();
}

/**
 * @brief Create junction between line and ellipse search lines.
 * @param sline Is the search line which triggered the new intersections.
 * @param iscts Intersection as vote image element
 */
void FormEJunctions::CreateJunction(unsigned sline, VoteImage::Elem &isct)
{
// printf("FormEJunctions::CreateJunction: sline: %u / iscts: %u\n", sline, isct.id);
  unsigned i = sline/core->VI()->GetBaseIndex();					// line number
  unsigned vtype_i = sline%core->VI()->GetBaseIndex();		// type of vote line

// printf("  i=%u / vtype_i=%u\n", i, vtype_i);

	// sline type is a line search line
	if(vtype_i == VOTE_E || vtype_i == VOTE_TS || vtype_i == VOTE_TE)
	{
// printf("FormEJunctions::CreateJunction: VOTE_E / VOTE_TS / VOTE_TE!\n");
		unsigned vtype_j = isct.id%core->VI()->GetBaseIndex();

		if(core->VI()->IsctTypeAdmissible(vtype_i, vtype_j) == 3)			// 3 == e-junction type
		{
// printf("FormEJunctions::CreateJunction: Valid admissible!\n");
			Line *line_i = Lines(core, i);
			int end_i = VOTE_END(vtype_i);
			unsigned j = isct.id/core->VI()->GetBaseIndex();
			int end_j = VOTE_VERTEX(vtype_j);
			Ellipse *ell_j = Ellipses(core, j-core->VI()->GetEllOffset());

			if(NoEJunctionYet(line_i, ell_j))
			{
				if(vtype_i == VOTE_E)   // recalculate line_end, if vote line was edge itself
				{
					// intersection point und start/end point
					Vector2 isct;
					Vector2 start = line_i->point[START];
					Vector2 end = line_i->point[END];
					try
					{
						isct = LineIntersection(line_i->point[end_i], line_i->tang[end_i], ell_j->vertex[end_j], ell_j->dir);
					}
					catch (exception &e)     // lines do not intersect, carry on
					{
						printf("FormEJunctions::CreateJunction: Lines do not intersect exception.\n");
					}
					if(Length(start-isct) < Length(end-isct)) end_j = 0;
					else end_j = 1;
				}
				core->NewGestalt(GestaltPrinciple::FORM_E_JUNCTIONS, new EJunction(core, line_i, ell_j, end_i, end_j));
			}
// else printf("   Already e-junction found 2 \n");
		}
	}
	
	// sline type is a ellipse search line
	else if(vtype_i == VOTE_EOTL || vtype_i == VOTE_EOTR || vtype_i == VOTE_EITL || vtype_i == VOTE_EITR)
	{
// printf("FormEJunctions::CreateJunctions: VOTE_EOTL / VOTE_EOTR / VOTE_EITL / VOTE_EITR!\n");

		unsigned vtype_j = isct.id%core->VI()->GetBaseIndex();
		if(core->VI()->IsctTypeAdmissible(vtype_i, vtype_j) == 3)			// 3 == e-junction type
		{
// printf("FormEJunctions::CreateJunctions: valid!\n");

			int end_i = VOTE_VERTEX(vtype_i);
			Ellipse *ell_i = Ellipses(core, i-core->VI()->GetEllOffset());

			unsigned j = isct.id/core->VI()->GetBaseIndex();
			int end_j = VOTE_END(vtype_j);
			Line *line_j = Lines(core, j);

			if(NoEJunctionYet(line_j, ell_i))
			{
				if(vtype_j == VOTE_E)   // recalculate line_end, if vote line was edge itself
				{
						Vector2 isct;
						Vector2 start = line_j->point[START];
						Vector2 end = line_j->point[END];
						try
						{
							isct = LineIntersection(line_j->point[end_j], line_j->tang[end_j], ell_i->vertex[end_i], ell_i->dir);
						}
						catch (exception &e)   // lines do not intersect, carry on
						{
							printf("FormEJunctions::CreateJunction: Lines do not intersect exception.\n");
							cout << e.what() << endl;
						}
						if(Length(start-isct) < Length(end-isct)) end_j = 0;
						else end_j = 1;
				}
// else printf("  Already e-junction found!!!\n");
// // if(end_j != changed) printf("  LINE END CHANGED\n");
				core->NewGestalt(GestaltPrinciple::FORM_E_JUNCTIONS, new EJunction(core, line_j, ell_i, end_j, end_i));
			}
		}
// printf("FormEJunctions::CreateJunctions: VOTE_EOTL / VOTE_EOTR / VOTE_EITL / VOTE_EITR end!\n");
	}

	// sline type is a ???
	else if(vtype_i == VOTE_NONE)
		printf("FormJunctions::CreateJunction: VOTE_NONE exception!\n");
// 	else
// 	{
// 		printf("FormEJunctions::CreateJunction: Warning: Unknown search line type: vtype_i = %u\n", vtype_i);
// 	}
	
// printf("FormEJunctions::CreateJunction: end.\n");
}

/**
 * @brief Create junction between line and ellipse search lines.
 * @param sline Is the search line which triggered the new intersections.
 * @param iscts ???
 */
void FormEJunctions::CreateJunctions(unsigned sline, Array<VoteImage::Elem> iscts)
{
// printf("FormEJunctions::CreateJunctions: sline: %u / iscts: %u\n", sline, iscts[0].id);
	for(unsigned k = 0; k < iscts.Size(); k++)
		CreateJunction(sline, iscts[k]);
}

/**
 * @brief Check whether there is no E-jct with line and ellipse.
 * @param line Line
 * @param ellipse Ellipse
 * @return Returns true, if there is no E-junction yet with these line-ellipse combination.
 */
bool FormEJunctions::NoEJunctionYet(Line *line, Ellipse *ellipse)
{
// 	for(unsigned i=0; i<NumEJunctions(core); i++)
// 	{
// 		if(EJunctions(core, i)->ellipse->ID() == ellipse->ID() &&
// 			 EJunctions(core, i)->line->ID() == line->ID())
// 				return false;
// 	}
	
	for(int i=LEFT; i<=RIGHT; i++)
		for(unsigned j=0; j<ellipse->ejcts[i].Size(); j++)
			if(ellipse->ejcts[i][j]->line->ID() == line->ID())
				return false;
	
  return true;
}

/**
 * @brief Update all ellipse-junctions, when a new collinearity appears.
 * @param idx Index of new collinearity.
 */
void FormEJunctions::UpdateEJunctions(unsigned idx)
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		if(EJunctions(core, i)->IsColLine(Collinearities(core, idx)->line[0]) ||																	/// TODO Kann nicht funktionieren => Falsche Linie! (IsLine)
			 EJunctions(core, i)->IsColLine(Collinearities(core, idx)->line[1]))
			EJunctions(core, i)->UpdateColLines(EJunctions(core, i)->line, EJunctions(core, i)->lineEnd);
	}
}

/**
 * @brief Mask the bad results: Mask E-Junctions, where the ellipse is masked.
 */
void FormEJunctions::Mask()
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		EJunction *ejct = (EJunction*)core->RankedGestalts(Gestalt::E_JUNCTION, i);
		if(ejct->ellipse->IsMasked())
			ejct->Mask(10000);
	}
}


}

