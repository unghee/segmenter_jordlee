/**
 * @file Color.hh
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2007, 2010
 * @version 0.1
 * @brief Color class.
 */

#ifndef Z_COLOR_HH
#define Z_COLOR_HH

// #include "Namespace.hh"

namespace Z
{

/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char b; // Blue channel
    unsigned char g; // Green channel
    unsigned char r; // Red channel
    unsigned char a; // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


/**
 * @brief Enum ColorFormat
 */
enum ColorFormat
{
  GRAY8,
  RGB24,
  BGR24,
  MAX_COLOR_FMT
};

/**
 * @brief Class RGBColor
 */
class RGBColor
{
public:
  static const RGBColor black;						///< Color: black
  static const RGBColor white;						///< Color: white
  static const RGBColor red;							///< Color: red
  static const RGBColor green;						///< Color: green
  static const RGBColor blue;							///< Color: blue
  static const RGBColor cyan;							///< Color: cyan
  static const RGBColor magenta;					///< Color: magenta
  static const RGBColor yellow;						///< Color: yellow
  static const RGBColor dark_yellow;			///< Color: dark_yellow
  static const RGBColor brown;						///< Color: brown
  static const RGBColor coral;						///< Color: coral
  static const RGBColor maroon;						///< Color: maroon
	static const RGBColor xxx;

  unsigned char r, g, b;									///< color channel: red, green, blue

  RGBColor() {}
  RGBColor(unsigned char _r, unsigned char _g, unsigned char _b) : r(_r), g(_g), b(_b) {}
  bool operator==(RGBColor c) {return r == c.r && g == c.g && b == c.b;}
  unsigned char Grey() {return (unsigned char)(((int)r + (int)g + (int)b)/3);}
};

extern const char* ColorFormat2String(ColorFormat col);
extern ColorFormat String2ColorFormat(const char *str);
extern int DistSqr(RGBColor x, RGBColor y);
extern double Dist(RGBColor x, RGBColor y);

}

#endif

