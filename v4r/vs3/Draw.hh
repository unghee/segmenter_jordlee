/**
 * @file Draw.hh
 * @author Richtsfeld
 * @date May 2009
 * @version 0.1
 * @brief Drawing Gestalts to the OpenCv IplImage.
 **/


#ifndef Z_DRAW_HH
#define Z_DRAW_HH

#include <opencv2/highgui/highgui.hpp>
#include "Color.hh"
#include "v4r/vs3/Vector.hh"

namespace Z
{

extern RGBColor defColor;				///< Default color; set with SetColor()

// extern void SetFatLines(bool set);
// extern void SetBlackWhite(bool set);
/// Draw printf style text in the status bar.
// extern void PrintStatus(const char *format, ...);
// extern void DrawImageRGB24(const char *rgb24, int width, int height);
// extern void SetDrawImagesSolid();
// extern void SetDrawImagesLight();
// extern void SetDrawImagesDark();
// extern void SetClearImagesWhite();
// extern void SetClearImagesBlack();

extern void SetActiveDrawArea(IplImage *iI);


extern void SetColor(RGBColor col);

extern void DrawText2D(const char *text, double x, double y, RGBColor col = defColor);
extern void DrawPoint2D(double x, double y, RGBColor col = defColor);
extern void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col = defColor);
extern void DrawLine2D(double x1, double y1, double x2, double y2, unsigned char transparency, RGBColor col = defColor);
extern void DrawArrow(VEC::Vector2 start, VEC::Vector2 end, RGBColor col = defColor);
extern void DrawRect2D(double x1, double y1, double x2, double y2, RGBColor col = defColor);
extern void FillRect2D(double x1, double y1, double x2, double y2, RGBColor col = defColor);
extern void TransparentRect2D(double x1, double y1, double x2, double y2, RGBColor col = defColor);
extern void TransparentQuadrilateral2D(double x1, double y1, double x2, double y2,
		double x3, double y3, double x4, double y4, bool fill, unsigned char transparency, RGBColor col = defColor);
extern void DrawArc2D(double x, double y, double r, double start, double span, RGBColor col = defColor);
extern void FillArc2D(double x, double y, double r, double start, double span, RGBColor col = defColor);
extern void TransparentArc2D(double x, double y, double r, double start, double span, RGBColor col = defColor);
extern void DrawEllipse2D(double x, double y, double a, double b, double phi, RGBColor col = defColor);
extern void FillEllipse2D(double x, double y, double a, double b, double phi, RGBColor col = defColor);
extern void TransparentEllipse2D(double x, double y, double a, double b, double phi, RGBColor col = defColor);

}

#endif

