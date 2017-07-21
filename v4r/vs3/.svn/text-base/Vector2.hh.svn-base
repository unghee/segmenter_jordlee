/**
 * @file Vector2.hh
 * @author Zillich, Richtsfeld
 * @date 2004, 2010
 * @version 0.1
 * @brief 2D Vector class.
 */

#ifndef VEC_VECTOR2_HH
#define VEC_VECTOR2_HH

#include <iostream>
#include <sstream>
#include <cstdio>
#include <string>
#include <stdexcept>
#include <math.h>

namespace VEC
{

/**
 * @brief 2D vector class.
 */
class Vector2
{
public:
  double x;
  double y;

  Vector2();
  Vector2(double xx, double yy);
  void Set(double xi, double yi) {x = xi; y = yi;}
  Vector2& operator+=(const Vector2 &v);
  Vector2& operator-=(const Vector2 &v);
  Vector2& operator*=(double s);
  Vector2& operator/=(double s) throw (std::runtime_error);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();
  Vector2 Normal();
  Vector2 NormalClockwise();
  Vector2 NormalAntiClockwise();
};

Vector2 operator-(const Vector2 &v);
bool operator==(const Vector2 &a, const Vector2 &b);
bool operator!=(const Vector2 &a, const Vector2 &b);
Vector2 operator+(const Vector2 &a, const Vector2 &b);
Vector2 operator-(const Vector2 &a, const Vector2 &b);
Vector2 operator*(const double s, const Vector2 &v);
Vector2 operator*(const Vector2 &v, const double s);
Vector2 operator/(const Vector2 &v, const double s) throw (std::runtime_error);
double PolarAngle(const Vector2 &v);
double Length(const Vector2 &v);
Vector2 Normalise(const Vector2 &v);
double Dot(const Vector2 &a, const Vector2 &b);
double Cross(const Vector2 &a, const Vector2 &b);
bool LeftOf(const Vector2 &a, const Vector2 &b);
bool CounterClockwiseTo(const Vector2 &a, const Vector2 &b);
bool RightOf(const Vector2 &a, const Vector2 &b);
bool ClockwiseTo(const Vector2 &a, const Vector2 &b);
double DistanceSquare(const Vector2 &a, const Vector2 &b);
double Distance(const Vector2 &a, const Vector2 &b);
double DistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
double AbsDistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
Vector2 Rotate(const Vector2 &a, double phi);
Vector2 CircleCenter(const Vector2 &pi, const Vector2 &pj, const Vector2 &pk);
Vector2 MidPoint(const Vector2 &a, const Vector2 &b);
double SmallestAngle(const Vector2 &a, const Vector2 &b);
double OpeningAngle(const Vector2 &a, const Vector2 &b);

std::ostream& operator<<(std::ostream &os, const Vector2 &v);
std::istream& operator>>(std::istream &is, Vector2 &v) throw (std::runtime_error);
std::string&       operator<<(std::string &s, const Vector2 &v);
const std::string& operator>>(const std::string &s, Vector2 &v);

Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2) throw (std::runtime_error);
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw (std::runtime_error);
Vector2 LineIntersectionEpipolarLine(const Vector2 &p1, const Vector2 &d1, double &py, double *l1);
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2) throw (std::runtime_error);
// bool LineIntersectEpipolarLine(const Vector2 &a1, const Vector2 &a2, double y);

  
/**
 * @brief Fast constructor e.g. for constructing large arrays of vectors.
 * values are uninitialised
 */
inline Vector2::Vector2()
{
}

inline Vector2::Vector2(double xx, double yy) : x(xx), y(yy)
{
}

inline Vector2& Vector2::operator+=(const Vector2 &v)
{
  x += v.x;
  y += v.y;
  return *this;
}

inline Vector2& Vector2::operator-=(const Vector2 &v)
{
  x -= v.x;
  y -= v.y;
  return *this;
}

inline Vector2& Vector2::operator*=(double s)
{
  x *= s;
  y *= s;
  return *this;
}

inline Vector2& Vector2::operator/=(double s) throw (std::runtime_error)
{
  if(s == 0)
    throw std::runtime_error("Vector2::operator/= : division by zero");
  x /= s;
  y /= s;
  return *this;
}

inline double Vector2::NormSquare() const
{
  return x*x + y*y;
}

inline double Vector2::LengthSquare() const
{
  return NormSquare();
}

inline double Vector2::Norm() const
{
  return sqrt(x*x + y*y);
}

inline double Vector2::Length() const
{
  return Norm();
}

inline void Vector2::Normalise()
{
  double n = Norm();
  if(n!=0.0f)
    *this /= n;
  else
    printf("[Vector2::Normalise] Warning: Norm is zero.\n");
}

inline Vector2 Vector2::Normal()
{
  return Vector2(-y, x);
}

/**
 * @brief Clockwise (mathematically negative) rotation, i.e. rotation to the right.
 * ATTENTION: For typical image co-ordinate systems with x-axis pointiing to
 * the right and y-axis pointing downwards things are reversed: clockwise
 * rotation in this case means rotation to the left.
 */
inline Vector2 Vector2::NormalClockwise()
{
  return Vector2(y, -x);
}

/**
 * @brief Anti-clockwise (mathematically positive) rotation, i.e. rotation to the left.
 * ATTENTION: For typical image co-ordinate systems with x-axis pointiing to
 * the right and y-axis pointing downwards things are reversed: anti-clockwise
 * rotation in this case means rotation to the right.
 */
inline Vector2 Vector2::NormalAntiClockwise()
{
  return Vector2(-y, x);
}

inline Vector2 operator-(const Vector2 &v)
{
  return Vector2(-v.x, -v.y);
}

inline bool operator==(const Vector2 &a, const Vector2 &b)
{
  return (fabs(a.x - b.x) < 1e-12 && fabs(a.y - b.y) < 1e-12);
}

inline bool operator!=(const Vector2 &a, const Vector2 &b)
{
  return !operator==(a, b);
}

inline Vector2 operator+(const Vector2 &a, const Vector2 &b)
{
  return Vector2(a.x + b.x, a.y + b.y);
}

inline Vector2 operator-(const Vector2 &a, const Vector2 &b)
{
  return Vector2(a.x - b.x, a.y - b.y);
}

inline Vector2 operator*(const double s, const Vector2 &v)
{
  return Vector2(v.x*s, v.y*s);
}

inline Vector2 operator*(const Vector2 &v, const double s)
{
  return s*v;
}

inline Vector2 operator/(const Vector2 &v, const double s) throw (std::runtime_error)
{
  if(s == 0)
    throw std::runtime_error("Vector2::operator/= : division by zero");
  return Vector2(v.x/s, v.y/s);
}

inline double PolarAngle(Vector2 &v)
{
  return atan2(v.y, v.x);
}

inline double Length(const Vector2 &v)
{
  return v.Norm();
}

inline Vector2 Normalise(const Vector2 &v)
{
  double n = v.Norm();
  return v/n;
}

inline double Dot(const Vector2 &a, const Vector2 &b)
{
  return a.x*b.x + a.y*b.y;
}

/**
 * @brief Vector cross product.
 * note: positive cross product a x b means b counterclockwise (i.e. left) to a
 */
inline double Cross(const Vector2 &a, const Vector2 &b)
{
  return a.x*b.y - a.y*b.x;
}

/**
 * @brief Returns true if b is left of a.
 * Same as counterclockwise.
 */
inline bool LeftOf(const Vector2 &a, const Vector2 &b)
{
  return Cross(a, b) > 0.;
}

/**
 * @brief Returns true if b is counterclockwise to a.
 * Same as left of.
 */
inline bool CounterClockwiseTo(const Vector2 &a, const Vector2 &b)
{
  return Cross(a, b) > 0.;
}

/**
 * @brief Returns true if b is right of a.
 * Same as clockwise.
 */
inline bool RightOf(const Vector2 &a, const Vector2 &b)
{
  return Cross(a, b) < 0.;
}

/**
 * @brief Returns true if b is clockwise to a.
 * Same as right of.
 */
inline bool ClockwiseTo(const Vector2 &a, const Vector2 &b)
{
  return Cross(a, b) < 0.;
}

inline double DistanceSquare(const Vector2 &a, const Vector2 &b)
{
  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}

inline double Distance(const Vector2 &a, const Vector2 &b)
{
  double disSqr = DistanceSquare(a, b);
  return sqrt(disSqr);
//  return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y));
}

/**
 * @brief Calculate center of circle from 3 points.
 * note: throws an exception if center cannot be calculated.
 */
inline Vector2 CircleCenter(const Vector2 &pi, const Vector2 &pj, const Vector2 &pk)
{
  // throws an exception if intersection cannot be calculated
  return LineIntersection(
      Vector2((pi.x + pj.x)/2., (pi.y + pj.y)/2.),
      Vector2(pj.y - pi.y, pi.x - pj.x),
      Vector2((pj.x + pk.x)/2., (pj.y + pk.y)/2.),
      Vector2(pk.y - pj.y, pj.x - pk.x));
}

/**
 * @brief Returns signed distance of point q from line defined by point p and unit
 * direction vector d.
 */
inline double DistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d)
{
  Vector2 p_to_q = q - p;
  return Cross(p_to_q, d);
}

inline double AbsDistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d)
{
  return fabs(DistPointToLine(q, p, d));
}

inline Vector2 Rotate(const Vector2 &a, double phi)
{
  double si = sin(phi), co = cos(phi);
  return Vector2(co*a.x - si*a.y, si*a.x + co*a.y);
}

inline Vector2 MidPoint(const Vector2 &a, const Vector2 &b)
{
  return Vector2((a.x + b.x)/2., (a.y + b.y)/2.);
}

/**
 * @brief Calcualates the opening angle between two vectors (0-PI)
 */
inline double OpeningAngle(const Vector2 &a, const Vector2 &b)
{
  return acos(Dot(Normalise(a), Normalise(b)));
}

/**
 * @brief Calcualates the smallest angle between two vectors (intsecting lines).
 */
inline double SmallestAngle(const Vector2 &a, const Vector2 &b)
{
  double angle = OpeningAngle(a, b);
  if(angle > M_PI/2.) angle = M_PI - angle;
  return angle;
}

/**
 * @brief Print a vector to a stream: '[x y]`
 */
inline std::ostream& operator<<(std::ostream &os, const Vector2 &v)
{
  return os << '[' << v.x << ' ' << v.y <<  ']';
}

/**
 * @brief Read a vector from a stream.
 * The expected format is: '[x y]', white spaces are ignored.
 */
inline std::istream& operator>>(std::istream &is, Vector2 &v) throw (std::runtime_error)
{
  char c;
  is >> c;
  if(c == '[')
  {
    is >> v.x >> v.y >> c;
    if(c != ']')
      throw std::runtime_error("Vector2::operator>> : error reading Vector2: ']' expected.");
  }
  else
    throw std::runtime_error("Vector2::operator>> : error reading Vector2: '[' expected.");
  return is;
}

/**
 * @brief Append a vector to a string: '[x y]`
 * Actually You should use stringstreams instead.
 */
inline std::string& operator<<(std::string &s, const Vector2 &v)
{
  std::stringstream ss;
  ss << v;
  s += ss.str();
  return s;
}

/**
 * @brief Reads a point from the BEGINNING of a string. Subsequent calls of operator<<
 * will always return the same point.
 * The expected format is: '[x y]', white spaces are ignored.
 * Actually You should use stringstreams instead.
 */
inline const std::string& operator>>(const std::string &s, Vector2 &v)
{
  std::stringstream ss(s);
  ss >> v;
  return s;
}



/**
 * @brief Returns intersection of lines defined by point p and direction d.
 * @param p1 Point on first line
 * @param d1 Direction of first line (unit vector!)
 * @param p2 Point on second line
 * @param d2 Direction of second line (unit vector!)
 * @return Returns the intersect point.
 */
inline Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2) throw (std::runtime_error)
{
  double d = Cross(d2, d1);
  if(d == 0.)    
    throw std::runtime_error("Vector2::LineIntersection: lines do not intersect exception");
  double l = Cross(d2, p2 - p1)/d;
  return Vector2(p1.x + l*d1.x, p1.y + l*d1.y);
}

/**
 * @brief Returns intersection of lines defined by point p and direction d.
 * @param p1 Point on first line
 * @param d1 Direction of first line (unit vector!)
 * @param p2 Point on second line
 * @param d2 Direction of second line (unit vector!)
 * @param l1 and l2 contain the lengths along the lines where the intersection lies,
 * measured in lengths of d1 and d2. So if you want l1 and l2 in pixels, d1 and d2 must be unit vectors.
 * @param l2 see l1
 * @return Returns the intersection point.
 */
inline Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw (std::runtime_error)
{
  double d = Cross(d2, d1);
  if(d == 0.)    
    throw std::runtime_error("Vector2::LineIntersection: lines do not intersect exception");
  Vector2 p12 = p2 - p1;
  *l1 = Cross(d2, p12)/d;
  *l2 = Cross(d1, p12)/d;
  return Vector2(p1.x + *l1*d1.x, p1.y + *l1*d1.y);
}

/**
 * @brief Returns the intersection of a line defined by point p and direction d with a epipolar line
 * on a certain line
 * @param p1 Point on first line
 * @param d1 Direction of first line (unit vector!)
 * @param py Y-coordinate of the epipolar line.
 * @param l1 contains the lengths along the lines where the intersection lies,
 * measured in lengths of d1. So if You want l1 in pixels, d1 must be unit vectors.
 * @return Returns the intersection point.
 */
inline Vector2 LineIntersectionEpipolarLine(const Vector2 &p1, const Vector2 &d1, double &py, double *l1)
{
  Vector2 p2(0., py);
  Vector2 d2(1., 0.);
  double l2;
  Vector2 result;

  try
  {
    result = LineIntersection(p1, d1, p2, d2, l1, &l2); 
  }
  catch(std::exception &e)
  {
    result.x = -1.;		// no result
    result.y = -1.;
  }
// printf("LineIntersectionEpipolarLine 1: %4.2f / %4.2f\n", xx.x, xx.y);
  return result;
}

/**
 * @brief Checks wheter lines a and b defined by their end points intersect.
 * @param a1 Start point of first line.
 * @param a2 End point of first line.
 * @param b1 Start point of second line.
 * @param b2 End point of second line.
 * @return Returns true, when the lines intersect.
 */
inline bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2) throw (std::runtime_error)
{
  double l1 = 0., l2 = 0.;
  LineIntersection(a1, a2 - a1, b1, b2 - b1, &l1, &l2);
  return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);
}

/**
 * @brief Checks wheter lines a and b defined by their end points intersect.
 * @param a1 Start point of line.
 * @param a2 End point of line.
 * @param y Y-coordinate of the epipolar line
 * @return Returns true, when the lines intersect.
 */
// inline bool LineIntersectEpipolarLine(const Vector2 &a1, const Vector2 &a2, double y)
// {
//   double l1 = 0., l2 = 0.;
//   Vector2 b1, d1;
//   b1.y = y; b1.x = 0;
//   d1.y = 1; d1.x = 0;
//   bool gotException = false;
//   try
//   {
// printf("#########  LineIntersectEpipolarLine: intersection!\n");
//     LineIntersection(a1, a2 - a1, b1, d1, &l1, &l2);
//   }
//   catch (std::exception e)
//   {
// printf("LineIntersectEpipolarLine: no intersection!\n");
//     gotException = true;
//     return false;
//   }
//   printf("LineIntersectEpipolarLine: GotException: %u\n", gotException);
//   return true;
// //   return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);		// l1 and l2 between 0 and 1
// }




}

// #include "Vector2.ic"
#endif

