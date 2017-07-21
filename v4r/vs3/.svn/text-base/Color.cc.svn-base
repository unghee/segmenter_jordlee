/**
 * @file Color.cc
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Color class.
 */

#include <string.h>
#include "v4r/vs3/Math.hh"
#include "Color.hh"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char names[][NAME_LENGTH] = {
  "GRAY8",
  "RGB24",
  "BGR24",
  "MAX_COLOR_FMT"
   };

// predefined colors
const RGBColor RGBColor::black(0, 0, 0);
const RGBColor RGBColor::white(255, 255, 255);
const RGBColor RGBColor::red(255, 0, 0);
const RGBColor RGBColor::green(0, 255, 0);
const RGBColor RGBColor::blue(0, 0, 255);
const RGBColor RGBColor::cyan(0, 255, 255);
const RGBColor RGBColor::magenta(255, 0, 255);
const RGBColor RGBColor::yellow(255, 255, 0);
const RGBColor RGBColor::dark_yellow(128, 128, 0);
const RGBColor RGBColor::brown(165,42,42);
const RGBColor RGBColor::coral(255,128,80);
const RGBColor RGBColor::maroon(152,0,0);
const RGBColor RGBColor::xxx(123,38,18);

/**
 * @brief Return color format as string.
 * @param col Color format
 * @return Color format as string.
 */
const char *ColorFormat2String(ColorFormat col)
{
  return names[col];
}


/**
 * @brief Return color format from string.
 * @param str Color format as string
 * @return Color format
 */
ColorFormat String2ColorFormat(const char *str)
{
  for(int i = 0; i < MAX_COLOR_FMT; i++)
    if(strncmp(str, names[i], NAME_LENGTH) == 0)
      return (ColorFormat)i;
  return MAX_COLOR_FMT;
}


/**
 * @brief Calculate square sum of color distance from two colors.
 * @param x First rgb-color.
 * @param y Second rgb-color.
 * @return Square sum of color distance as integer.
 */
int DistSqr(RGBColor x, RGBColor y)
{
  return Sqr((int)x.r - (int)y.r) + Sqr((int)x.g - (int)y.g) + Sqr((int)x.b - (int)y.b);
}

/**
 * @brief Calculate color distance from two colors.
 * @param x First rgb-color.
 * @param y Second rgb-color.
 * @return Color distance as double value.
 */
double Dist(RGBColor x, RGBColor y)
{
  return sqrt((double)DistSqr(x, y));
}

}

