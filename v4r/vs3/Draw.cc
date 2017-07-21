/**
 * @file Draw.cc
 * @author Richtsfeld Andreas, Zillich Michael
 * @date 2007, May 2009, 2010
 * @version 0.1
 * @brief Drawing Gestalts to the OpenCv IplImage.
 **/


#include "Draw.hh"
#include <math.h>

namespace Z
{
																						/// TODO iplImage is global
IplImage *iplImage;													///< Draw image in Draw.cc
RGBColor defColor = RGBColor::white;				///< Default draw color in Draw.cc
int drawThickness = 2;											///< Default draw thickness in Draw.cc

/**
 * @brief Set the IplImage as active draw area.
 * @param iI OpenCv IplImage for drawings.
 */
void SetActiveDrawArea(IplImage *iI)
{
	iplImage = iI;
}

/**
 * Set/unset fat lines and dots for all further drawing.
 * Note: only affects the main draw area;
 */
// void SetFatLines(bool set)
// {
// //   THICK_LINES = set;
// }

/**
 * Set/unset black and white drawing of lines, dots etc. for all further
 * drawing.
 * Note: only affects the main draw area.
 */
// void SetBlackWhite(bool set)
// {
// //   BLACK_WHITE = set;
// }

/**
 * Draw images solid.
 */
// void SetDrawImagesSolid()
// {
//   glPixelTransferf(GL_RED_SCALE, 1.);
//   glPixelTransferf(GL_RED_BIAS, 0.);
//   glPixelTransferf(GL_GREEN_SCALE, 1.);
//   glPixelTransferf(GL_GREEN_BIAS, 0.);
//   glPixelTransferf(GL_BLUE_SCALE, 1.);
//   glPixelTransferf(GL_BLUE_BIAS, 0.);
// }

/**
 * Draw images light transparent.
 * Good as background for displaying edges etc.
 */
// void SetDrawImagesLight()
// {
//   float f = 0.4;
//   glPixelTransferf(GL_RED_SCALE, f);
//   glPixelTransferf(GL_RED_BIAS, 1. - f);
//   glPixelTransferf(GL_GREEN_SCALE, f);
//   glPixelTransferf(GL_GREEN_BIAS, 1. - f);
//   glPixelTransferf(GL_BLUE_SCALE, f);
//   glPixelTransferf(GL_BLUE_BIAS, 1. - f);
// }

/**
 * Draw images dark transparent.
 * Good as background for displaying edges etc.
 */
// void SetDrawImagesDark()
// {
//   float f = 0.4;
//   glPixelTransferf(GL_RED_SCALE, f);
//   glPixelTransferf(GL_RED_BIAS, 0.);
//   glPixelTransferf(GL_GREEN_SCALE, f);
//   glPixelTransferf(GL_GREEN_BIAS, 0.);
//   glPixelTransferf(GL_BLUE_SCALE, f);
//   glPixelTransferf(GL_BLUE_BIAS, 0.);
// }


/**
 * @brief Set default color for drawing (if no color is explicitely given).
 */
void SetColor(RGBColor col)
{
	defColor = col;
}

/**
 * @brief Draw 2D text.
 * @param text Text
 * @param x x-position
 * @param y y-position
 * @param col Color of Text
 */
void DrawText2D(const char *text, double x, double y, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

	cvPutText (iplImage, text, cvPoint(x, y), &font, cvCol);
}

/**
 * @brief Draw a 2D point.
 */
void DrawPoint2D(double x, double y, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x, y), cvPoint(x,y), cvCol, 2);
}

/**
 * @brief Draw a 2D line.
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y2), cvCol, drawThickness);
}

/**
 * @brief Draw a 2D line.
 * @TODO Without transparency
 */
void DrawLine2D(double x1, double y1, double x2, double y2, unsigned char transparency, RGBColor col)
{
	DrawLine2D(x1, y1, x2, y2, col);
}

/**
 * @brief Draws an arrow at the middle of a line.
 * @param start Start point of line.
 * @param end End point of line.
 * @param color Color of Arrow.
 */
void DrawArrow(VEC::Vector2 start, VEC::Vector2 end, RGBColor col)
{
  double len_2 = 4.;
  double wid_2 = 3.;
	VEC::Vector2 dir = end - start;
	dir = Normalise(dir);
  VEC::Vector2 tip = (start + end)/2. + dir*len_2;
  VEC::Vector2 left = (start + end)/2. - dir*len_2 + dir.NormalAntiClockwise()*wid_2;
  VEC::Vector2 right = (start + end)/2. - dir*len_2 + dir.NormalClockwise()*wid_2;
  DrawLine2D(left.x, left.y, tip.x, tip.y, col);
  DrawLine2D(right.x, right.y, tip.x, tip.y, col);
}

/**
 * @brief Draw a 2D rectangle.
 * @TODO Without transparency
 */
static void Rect2D(double x1, double y1, double x2, double y2, bool fill, unsigned char transparency, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	int thickness = drawThickness;
	if(fill) thickness = -1;

	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y1), cvCol, thickness);
	cvLine(iplImage, cvPoint(x2, y1), cvPoint(x2,y2), cvCol, thickness);
	cvLine(iplImage, cvPoint(x2, y2), cvPoint(x1,y2), cvCol, thickness);
	cvLine(iplImage, cvPoint(x1, y2), cvPoint(x1,y1), cvCol, thickness);
}

/**
 * @brief Draw a 2D rectangle.
 */
void DrawRect2D(double x1, double y1, double x2, double y2, RGBColor col)
{
  Rect2D(x1, y1, x2, y2, false, 255, col);
}

/**
 * @brief Draw a filled 2D rectangle.
 */
void FillRect2D(double x1, double y1, double x2, double y2, RGBColor col)
{
  Rect2D(x1, y1, x2, y2, true, 255, col);
}

/**
 * @brief Draw a half transparent 2D rectangle.
 */
void TransparentRect2D(double x1, double y1, double x2, double y2, RGBColor col)
{
  Rect2D(x1, y1, x2, y2, true, 64, col);
}

/**
 * @brief Draw a quadrilateral with transparency
 * @TODO Without transparency
 */
void TransparentQuadrilateral2D(double x1, double y1, double x2, double y2,		// HACK ARI
    double x3, double y3, double x4, double y4, bool fill, unsigned char transparency, RGBColor col)
{
	int thickness = drawThickness;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y2), cvCol, thickness, CV_AA);
	cvLine(iplImage, cvPoint(x2, y2), cvPoint(x3,y3), cvCol, thickness, CV_AA);
	cvLine(iplImage, cvPoint(x3, y3), cvPoint(x4,y4), cvCol, thickness, CV_AA);
	cvLine(iplImage, cvPoint(x4, y4), cvPoint(x1,y1), cvCol, thickness, CV_AA);
}


/**
 * @brief Draw an arc with center (x,y), radius r from angle start to end.
 * @TODO Without transparency
 */
static void Arc2D(double x, double y, double r, double start, double span, bool fill, unsigned char transparency, RGBColor col)
{
	int thickness = drawThickness;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);

	if(r > 30000.) return;
	cvEllipse(iplImage, cvPoint(x, y), cvSize(r, r), (-start)*360/(2*M_PI), 0, (-span)*360/(2*M_PI), cvCol, thickness);
}

/**
 * @brief Draw an arc with center (x,y), radius r from angle start to end.
 */
void DrawArc2D(double x, double y, double r, double start, double span, RGBColor col)
{
  Arc2D(x, y, r, start, span, false, 255, col);
}

/**
 * @brief Fill an arc with center (x,y), radius r from angle start to end.
 */
void FillArc2D(double x, double y, double r, double start, double span, RGBColor col)
{
  Arc2D(x, y, r, start, span, true, 255, col);
}

/**
 * @brief Fill an arc with center (x,y), radius r from angle start to end using
 * transparent color.
 */
void TransparentArc2D(double x, double y, double r, double start, double span, RGBColor col)
{
  Arc2D(x, y, r, start, span, true, 64, col);
}

/**
 * @brief Approximate ellipse with 100 lines.
 * Note: fill and transparency ignored for now
 */
static void Ellipse2D(double x, double y, double a, double b, double phi, bool fill, unsigned char transparency, RGBColor col)
{
	int thickness = drawThickness;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);

	cvEllipse(iplImage, cvPoint(x, y), cvSize(a, b), -phi*360/(2*M_PI), 0, 360, cvCol, thickness);
}

/**
 * @brief Draw a 2D Ellipse.
 */
void DrawEllipse2D(double x, double y, double a, double b, double phi, RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, false, 255, col);
}


/**
 * @brief Draw a 2D Ellipse.
 */
void FillEllipse2D(double x, double y, double a, double b, double phi, RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, true, 255, col);
}

/**
 * @brief Draw a 2D Ellipse.
 */
void TransparentEllipse2D(double x, double y, double a, double b, double phi, RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, true, 64, col);
}

}

