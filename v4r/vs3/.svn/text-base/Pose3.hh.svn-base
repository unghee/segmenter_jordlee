/**
 * Minimalistic classes for doing necessary linear algebra stuff.
 */

#ifndef VEC_POSE3_HH
#define VEC_POSE3_HH

#include <cstdio>
#include <iostream>
#include <cstdio>
#include <algorithm>
#include "Vector3.hh"

namespace VEC
{

class Pose3
{
public:
  Vector3 pos;
  Vector3 rot;

  Pose3() {}
  Pose3(const Vector3 &_pos, const Vector3 &_rot) : pos(_pos), rot(_rot) {}
  Pose3 Inverse() const;
  Vector3 Transform(const Vector3 &p) const;
  Vector3 TransformDir(const Vector3 &d) const;
  void ToMat44(double M[4][4]) const;
};

inline void TransposeMat44(double M[4][4])
{
  for(int i = 0; i < 4; i++)
    for(int j = i+1; j < 4; j++)
      std::swap(M[i][j], M[j][i]);
}

inline void Identity3x3(double M[3][3])
{
  int i, j;
  for(i = 0; i < 3; i++)
    for(j = 0; j < 3; j++)
      M[i][j] = 0.;
  M[0][0] = M[1][1] = M[2][2] = 1.;
}

inline void RotationAxisAngleToMatrix(const Vector3 &r, double R[3][3])
{
  double th = Length(r);
  if(th != 0)
  {
    double x = r.x/th, y = r.y/th, z = r.z/th;
    double co = cos(th), si = sin(th);
    R[0][0] = x*x*(1. - co) + co;
    R[0][1] = x*y*(1. - co) - z*si;
    R[0][2] = x*z*(1. - co) + y*si;
    R[1][0] = x*y*(1. - co) + z*si;
    R[1][1] = y*y*(1. - co) + co;
    R[1][2] = y*z*(1. - co) - x*si;
    R[2][0] = x*z*(1. - co) - y*si;
    R[2][1] = y*z*(1. - co) + x*si;
    R[2][2] = z*z*(1. - co) + co;
  }
  else
  {
    Identity3x3(R);
  }
}

inline void RotationMatrixToAxisAngle(const double R[3][3], Vector3 &r)
{
//   double trace = R[0][0] + R[1][1] + R[2][2];
//   double angle = acos((trace - 1.) / 2.);
//   Real s = 2. * sin(angle);
//   if (isnormal(s))
//   {
//     r.x = angle*(R[2][1] - R[1][2])/s;
//     r.y = angle*(R[0][2] - R[2][0])/s;
//     r.z = angle*(R[1][0] - R[0][1])/s;
//   }
//   else
//   {
//     r.x = 0.;
//     r.y = 0.;
//     r.z = 0.;
//   }

	printf("Error: VecMath.hh: RotationMatrixToAxisAngle: function not available.\n");
}

/**
 * Rotate vector p according to rotation vector r. Axis is defined by direction
 * of r and angle is defined by length of r.
 */
inline Vector3 Rotate(const Vector3 &r, const Vector3 &p)
{
  double R[3][3];
  Vector3 q;
  RotationAxisAngleToMatrix(r, R);
  q.x = R[0][0]*p.x + R[0][1]*p.y + R[0][2]*p.z;
  q.y = R[1][0]*p.x + R[1][1]*p.y + R[1][2]*p.z;
  q.z = R[2][0]*p.x + R[2][1]*p.y + R[2][2]*p.z;
  return q;
}

/**
 * Returns the inverse pose.
 *
 * w = R o + t
 * o = R^T w - R^T t
 */
inline Pose3 Pose3::Inverse() const
{
  Pose3 inv;
  inv.rot = -rot;
  inv.pos = -Rotate(inv.rot, pos);
  return inv;
}

/**
 * Transform a point from local (object) to global (world) coordinates.
 */
inline Vector3 Pose3::Transform(const Vector3 &p) const
{
  return pos + Rotate(rot, p);
}

/**
 * Transform a direction vector from local (object) to global (world)
 * coordinates.
 */
inline Vector3 Pose3::TransformDir(const Vector3 &d) const
{
  return Rotate(rot, d);
}

inline void Pose3::ToMat44(double M[4][4]) const
{
  double R[3][3];
  RotationAxisAngleToMatrix(rot, R);
  // upper 3x3
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      M[i][j] = R[i][j];
  // first 3 elements of last row
  for(int j = 0; j < 3; j++)
    M[3][j] = 0.;
  // last column
  M[0][3] = pos.x;
  M[1][3] = pos.y;
  M[2][3] = pos.z;
  M[3][3] = 1.;
}

inline std::istream& operator>>(std::istream &is, Pose3 &p)
{
  return is >> p.pos >> p.rot;
}

inline std::ostream& operator<<(std::ostream &is, const Pose3 &p)
{
  return is << p.pos << "   " << p.rot;
}

}

#endif

