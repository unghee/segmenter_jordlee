/**
 * $Id: ExtEllipse.hh,v 1.0 2007/12/06 13:47:03 mxz Exp mxz $
 */

#ifndef Z_EXTELLIPSE_HH
#define Z_EXTELLIPSE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class ExtEllipse : public Gestalt
{
public:
//   unsigned ellipse;										// ellipse
//   double x, y, a, b, phi;  						// ellipse parameters															// TODO gibt es nicht mehr! sinnlos, weil in ellipse
//   Vector2 vertex[2]; 									// vertex [LEFT/RIGHT]														// TODO sinnlos, weil in ellipse
//   Vector2 dir;         								// direction between vertices, normalised to 1		// TODO sinnlos, weil in ellipse
//   Array<unsigned> extLines;						// extended lines of the ellipse
//   Array<unsigned> extLinesEnd; 				// START/END point is near vertex
//   Array<unsigned> extLinesVertex; 		// at LEFT/RIGHT vertex of ellipse
// 	Array<double> extLinesGap;					// TODO gap between extLine and ellipse vertex
//   Array<unsigned> colLines;		
//   Array<unsigned> colLinesEnd;
//   Array<unsigned> colLinesVertex;
// 	Array<double> colLinesGap;					// TODO gap between colLine and ???

  ExtEllipse(unsigned eJct);
  void ExtendExtEllipse(unsigned eJct);
  void CalculateCollLines();
  void ColOfCol(unsigned line, Array<unsigned> *colOfColLines, Array<double> *gapColOfColLines);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
//   void DrawArrow();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& ExtEllipses()
{
  return Gestalts(Gestalt::EXT_ELLIPSE);
}
inline ExtEllipse* ExtEllipses(unsigned id)
{
  return (ExtEllipse*)Gestalts(Gestalt::EXT_ELLIPSE, id);
}
inline unsigned NumExtEllipses()
{
  return NumGestalts(Gestalt::EXT_ELLIPSE);
}

}

#endif
