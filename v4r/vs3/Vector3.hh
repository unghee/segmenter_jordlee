/**
 * @file Vector3.hh
 * @author Zillich, Richtsfeld
 * @date 2006, 2010
 * @version 0.1
 * @brief 3D Vector class.
 */

#ifndef VEC_VECTOR3_HH
#define VEC_VECTOR3_HH

#include <iostream>
#include <stdexcept>

namespace VEC
{

/**
 * @brief 3D vector class.
 */
class Vector3
{
public:
  double x;
  double y;
  double z;

  Vector3() {}
  Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
  Vector3& operator+=(const Vector3 &v);
  Vector3& operator-=(const Vector3 &v);
  Vector3& operator*=(double s);
  Vector3& operator/=(double s) throw (std::runtime_error);

  double Norm() const;
  bool Normalise();
};

Vector3 operator-(const Vector3 &v);
Vector3 operator+(const Vector3 &a, const Vector3 &b);
Vector3 operator-(const Vector3 &a, const Vector3 &b);
Vector3 operator*(const double s, const Vector3 &v);
Vector3 operator*(const Vector3 &v, const double s);
Vector3 operator/(const Vector3 &v, const double s) throw (std::runtime_error);
double Norm(const Vector3 &v);
double Length(const Vector3 &v);
double Dot(const Vector3 &a, const Vector3 &b);
Vector3 Cross(const Vector3 &a, const Vector3 &b);
Vector3 Normalise(const Vector3 &v);
double OpeningAngle(const Vector3 &a, const Vector3 &b);
double SmallestAngle(const Vector3 &a, const Vector3 &b);

inline std::istream& operator>>(std::istream &is, Vector3 &v);
inline std::ostream& operator<<(std::ostream &os, const Vector3 &v);

}

#include "Vector3.ic"

#endif

