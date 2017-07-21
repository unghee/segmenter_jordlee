/**
 * @file EJunction.cc
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt Ellipse-Junction (EJunction).
 **/

#include "EJunction.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt E-Junction.
 * @param vc Vision core
 * @param l Line of E-Junction
 * @param e Ellipse of E-Junction
 * @param lE Line End, which is farer away from E-junction
 */
EJunction::EJunction(VisionCore *vc, Line *l, Ellipse *e, unsigned lE, unsigned vtx) : Gestalt(vc, E_JUNCTION)
{
	line = l;
	ellipse = e;
  lineEnd = lE;
	vertex = vtx;
	
	// calculate intersection point:
	try
	{
		isct = LineIntersection(line->point[lineEnd], line->tang[lineEnd], ellipse->vertex[vertex], ellipse->dir, &gap[0], &gap[1]);
	}
	catch (exception &e)
  {
		printf("EJunction::EJunction: Lines do not intersect exception.\n");
    cout << e.what() << endl;
  }

	// get lines which are connected with the line via a collinearity
	UpdateColLines(line, Other(lineEnd));
	
	// Add e-junction to ellipsen
	ellipse->ejcts[vertex].PushBack(this);
	
//   CalculateSignificance();																																// TODO Calculate significance
}

/**
 * @brief Update the collinearity lines. Get all following lines, which are
 * connected to line via a collinearity.
 * @param l Line for which we are looking for new collinearities.
 * @param lE Line end, where we are searching for collinearities.
 */
void EJunction::UpdateColLines(Line *l, unsigned lE)
{
	// get all col-lines 
	for(unsigned i=0; i<l->coll[lE].Size(); i++)
	{
		Line *newColLine = l->coll[lE][i]->OtherLine(l);
		unsigned newColLineEnd = Other(l->coll[lE][i]->near_point[l->coll[lE][i]->WhichLineIs(newColLine)]);
		if(colLines.Find(newColLine) == UNDEF_ID)
		{
			colLines.PushBack(newColLine);
			colLinesEnd.PushBack(newColLineEnd);
			UpdateColLines(newColLine, newColLineEnd);
		}
	}
}

/**
 * @brief Draw the E-junction
 * @param detail Degree of detail.
 */
void EJunction::Draw(int detail)
{
	if(detail == 0)
	{
		DrawPoint2D(isct.x, isct.y);
	}
  else if(detail == 1)
  {
		ellipse->Draw();
		line->Draw();
		DrawPoint2D(isct.x, isct.y, RGBColor::blue);
	}
  else if(detail == 2)
  {
		ellipse->Draw(1);
		line->Draw(1);
		for(unsigned i=0; i<colLines.Size(); i++)
			colLines[i]->Draw(1);
	}
	else if(detail == 3)
  {
		Vector2 vtx = ellipse->vertex[vertex];
		Vector2 le = line->point[lineEnd];

		DrawLine2D(isct.x, isct.y, vtx.x, vtx.y, RGBColor::blue);
		DrawLine2D(isct.x, isct.y, le.x, le.y, RGBColor::blue);
  }
  else if(detail == 4)
  {
		ellipse->Draw(0);
		line->Draw(0);
		ellipse->DrawVotes();
		line->DrawVotes();							/// TODO TODO TODO Wie kann man die line votes zeichnen?
  }
  else if(detail >= 5)
  {
		ellipse->Draw(0);
		line->Draw(0);
		
		DrawPoint2D(isct.x, isct.y, RGBColor::blue);
		char id_str[20];
    snprintf(id_str, 20, "isct");
    DrawText2D(id_str, isct.x+5, isct.y, RGBColor::blue);
	}
}

/**
 * @brief Get Gestalt information.
 * @return Returns the Gestalt information as string.
 */
const char* EJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, "%s", Gestalt::GetInfo());

  n += snprintf(info_text + n, info_size - n, 
	"  ellipse: %u @ vertex: %u\n  line: %u @ end: %u", ellipse->ID(), vertex, line->ID(), lineEnd);
	
	n += snprintf(info_text + n, info_size - n, "\n  colLines: ");
	for (unsigned i=0; i<colLines.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", colLines[i]->ID());

	n += snprintf(info_text + n, info_size - n, "\n  colLinesEnd: ");
	for (unsigned i=0; i<colLines.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", colLinesEnd[i]);
  
	return info_text;
}

/**
 * @brief Checks, if the Gesalt is at given position.
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if Gestalt is at given position.
 */
bool EJunction::IsAtPosition(int x, int y)
{
  if (line->IsAtPosition(x, y) || ellipse->IsAtPosition(x, y))
		return true;
  return false;
}

/**
 * @brief Calculate the significance of the Gestalt.
 */
void EJunction::CalculateSignificance()
{
	// TODO implement
}

/**
 * @brief Returns true, if line is part of the e-junction.
 * @param l Line
 */
bool EJunction::IsLine(Line *l)
{
	if(line->ID() == l->ID()) return true;
	return false;
}

/**
 * @brief Returns true, if line is within the col-lines or is the e-junction line.
 * @param l Line to check
 * @return Returns true, if line is part of the ellipse junction.
 */
bool EJunction::IsColLine(Line *l)
{
	if(colLines.Contains(l) || l == line) return true;
	return false;
}

}





