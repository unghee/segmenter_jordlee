

#include <math.h>
#include "Draw.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "Ellipse.hh"
#include "EJunction.hh"
#include "ExtEllipse.hh"


namespace Z
{

ExtEllipse::ExtEllipse(unsigned eJct) : Gestalt(EXTELLIPSE)
{
//   ellipse = EJunctions(eJct)->ellipse;
// 
//   // push back lines, line-ends and vertex	
//   extLines.PushBack(EJunctions(eJct)->line);
//   extLinesEnd.PushBack(EJunctions(eJct)->lineEnd);
//   extLinesVertex.PushBack(EJunctions(eJct)->vertex);
// 	extLinesGap.PushBack(EJunctions(eJct)->gap[0] + EJunctions(eJct)->gap[1]);
// 
//   // add extEllipse in ellipse and line
//   Ellipses(ellipse)->AddExtEllipse(id);
//   Lines(EJunctions(eJct)->line)->AddExtEllipse(id);
// 
//   // ellipse parameters: x, y, a, b, phi
//   double params[5];
//   params[0] = Ellipses(ellipse)->x;
//   params[1] = Ellipses(ellipse)->y;
//   params[2] = Ellipses(ellipse)->a;
//   params[3] = Ellipses(ellipse)->b;
//   params[4] = Ellipses(ellipse)->phi;
// 
//   // vertices of the ellipse normalized to 0-Pi
//   vertex[LEFT] = Ellipses(ellipse)->vertex[LEFT];
//   vertex[RIGHT] = Ellipses(ellipse)->vertex[RIGHT];
// 
//   // normal direction of the ellipse
//   dir = Normalise(vertex[RIGHT] - vertex[LEFT]);
// 
//   CalculateCollLines();
}

/**
**	ExtendExtEllipse()
*/
void ExtEllipse::ExtendExtEllipse(unsigned eJct)
{
//   // push back lines, line-ends and vertex	
//   extLines.PushBack(EJunctions(eJct)->line);
//   extLinesEnd.PushBack(EJunctions(eJct)->lineEnd);
//   extLinesVertex.PushBack(EJunctions(eJct)->vertex);
// 	extLinesGap.PushBack(EJunctions(eJct)->gap[0] + EJunctions(eJct)->gap[1]);
// 
//   // register extEllipse in line	
//   Lines(EJunctions(eJct)->line)->AddExtEllipse(id);
// 	
//   CalculateCollLines();
}

/**
**	CalcualateCollLines()
*/
void ExtEllipse::CalculateCollLines()
{
// 	// clear collLines															// TODO ARI: sollten colLines wirklich gelöscht werden?
// 	colLines.Clear();
// 	colLinesEnd.Clear();
// 	colLinesVertex.Clear();
// 	colLinesGap.Clear();
// 
// 	// get every extLine
// 	for(unsigned i=0; i<extLines.Size(); i++)
// 	{
// 	  // get the collinearities from extLine
// 	  Array<unsigned> col;
// 	  Lines(extLines[i])->GetAllCollinearities(&col);
// 	
// 	  // every collinearity
// 	  for(unsigned j=0; j<col.Size(); j++)
// 	  {
// 			unsigned otherLine;
// 				
// 			// find other line of Colls
// 			if (Collinearities(col[j])->line[0] == extLines[i])
// 				otherLine = Collinearities(col[j])->line[1];
// 			else
// 				otherLine = Collinearities(col[j])->line[0];
// 	
// 			if (!extLines.Contains(otherLine) && !colLines.Contains(otherLine))
// 			{
// 				colLines.PushBack(otherLine);
// 				colLinesEnd.PushBack(extLinesEnd[i]);
// 				colLinesVertex.PushBack(extLinesVertex[i]);
// 				colLinesGap.PushBack(extLinesGap[i] + Collinearities(col[j])->gap);
// 
// 				Lines(otherLine)->AddExtEllipse(id);
// 			}
// 	  }
// 	
// 	  // get all Coll-Lines of collLines
// 	  for(unsigned i=0; i<colLines.Size(); i++)
// 	  {
// 			Array<unsigned> colOfColLines;
// 			Array<double> gapColOfColLines;
// 			ColOfCol(colLines[i], &colOfColLines, &gapColOfColLines);	  
// 				
// 			// colOfColLines auf colLines zurückkopieren!
// 			for(unsigned j=0; j<colOfColLines.Size(); j++)
// 				if(!colLines.Contains(colOfColLines[j]) &&	!extLines.Contains(colOfColLines[j]))
// 				{
// 					colLines.PushBack(colOfColLines[j]);
// 					colLinesEnd.PushBack(colLinesEnd[i]);			// TODO ARI: Richtig so?
// 					colLinesVertex.PushBack(colLinesVertex[i]);
// 					colLinesGap.PushBack(colLinesGap[i] + gapColOfColLines[j]);	
// 
// 					Lines(colOfColLines[j])->AddExtEllipse(id);
// 																					// TODO ARI: insert extEllipse to otherLine
// 				}
// 		}
// 	}
}

/**
**	ColOfCol()
*/
void ExtEllipse::ColOfCol(unsigned line, Array<unsigned> *colOfColLines, Array<double> *gapColOfColLines)
{
//   Array<unsigned> c = *colOfColLines;
// 	Array<double> g = *gapColOfColLines;
// 	
//   // get the collinearities from extLine i
//   Array<unsigned> col;
//   Lines(line)->GetAllCollinearities(&col);
// 
//   // every collinearity j
//   for(unsigned j=0; j<col.Size(); j++)
//   {
// 		unsigned otherLine;
// 			
// 		// find other line of Colls
// 		if (Collinearities(col[j])->line[0] == line)
// 			otherLine = Collinearities(col[j])->line[1];
// 		else
// 			otherLine = Collinearities(col[j])->line[0];
// 	
// 		if (!c.Contains(otherLine))
// 		{
// 			c.PushBack(otherLine);
// 			double gap = Collinearities(col[j])->gap;
// 			if(g.Size() > 0) gap += g[g.Size()-1];
// 			g.PushBack(gap);
// 			ColOfCol(otherLine, &c, &g);
// 		}
//   }
//   
//   *colOfColLines = c;
// 	*gapColOfColLines = g;
}

/**
**	DrawArrow()
*/
// void ExtEllipse::DrawArrow()
// {
//   double len_2 = 4.; // length/2
//   double wid_2 = 3.; // width/2
//   Vector2 tip = (vertex[START] + vertex[END])/2. + dir*len_2;
//   Vector2 left = (vertex[START] + vertex[END])/2. - dir*len_2 +
//     dir.NormalAntiClockwise()*wid_2;
//   Vector2 right = (vertex[START] + vertex[END])/2. - dir*len_2 +
//     dir.NormalClockwise()*wid_2;
//   DrawLine2D(left.x, left.y, tip.x, tip.y, RGBColor::blue);
//   DrawLine2D(right.x, right.y, tip.x, tip.y, RGBColor::blue);
// }

/**
**	Draw()
*/
void ExtEllipse::Draw(int detail)
{
//   Ellipses(ellipse)->Draw(detail);
// 	
//   for (unsigned i=0; i<extLines.Size(); i++)
// 		Lines(extLines[i])->Draw(detail-1);
// 
//   if (detail == 1) 
//   {
// 		for (unsigned i=0; i<colLines.Size(); i++)
// 		{
// 			Vector2 p0=Lines(colLines[i])->point[0];
// 			Vector2 p1=Lines(colLines[i])->point[1];
// 			DrawLine2D(p0.x, p0.y, p1.x, p1.y, RGBColor::blue);
// 		}	
//   }
// 
// 	if(detail == 2)
// 		for (unsigned i=0; i<colLines.Size(); i++)
// 			Lines(colLines[i])->Draw(detail-1);
// 
//   
//   if (detail == 3)  
//   {
// 		DrawLine2D(vertex[LEFT].x, vertex[LEFT].y, vertex[RIGHT].x, vertex[RIGHT].y, RGBColor::blue);
//   	DrawArrow();
//   }
}

/**
**	GetInfo()
*/
const char* ExtEllipse::GetInfo()
{
//   const unsigned info_size = 10000;
//   static char info_text[info_size] = "";
//   int n=0;
//   n += snprintf(info_text, info_size, "%s\nellipse: %u\nextLines: ",
//     Gestalt::GetInfo(), ellipse);
//   for (unsigned i=0; i<extLines.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", extLines[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\nextLinesEnd: ");
//   for (unsigned i=0; i<extLinesEnd.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", extLinesEnd[i]);
// 	  
//   n += snprintf(info_text + n, info_size - n, "\nextLinesVertex: ");
//   for (unsigned i=0; i<extLinesVertex.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", extLinesVertex[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\nextLinesGap: ");
//   for (unsigned i=0; i<extLinesGap.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%3.1f ", extLinesGap[i]);
// 	  
//   n += snprintf(info_text + n, info_size - n, "\ncolLines: ");
//   for (unsigned i=0; i<colLines.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", colLines[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\ncolLinesEnd: ");
//   for (unsigned i=0; i<colLinesEnd.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", colLinesEnd[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\ncolLinesVertex: ");
//   for (unsigned i=0; i<colLinesVertex.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%u ", colLinesVertex[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\ncolLinesGap: ");
//   for (unsigned i=0; i<colLinesGap.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n, "%3.1f ", colLinesGap[i]);
// 	  
// 
//   return info_text;
}

bool ExtEllipse::IsAtPosition(int x, int y)
{
//   for(unsigned i=0; i< extLines.Size(); i++)
//     if(Lines(extLines[i])->IsAtPosition(x, y))
//       return true;
//   return Ellipses(ellipse)->IsAtPosition(x, y);
}

void ExtEllipse::CalculateSignificance()
{
// 	sig = Ellipses(ellipse)->sig;
}

}
