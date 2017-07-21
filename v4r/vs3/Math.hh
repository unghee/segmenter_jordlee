/**
 * @file Math.hh
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2004, 2010, 2011
 * @version 0.2
 * @brief Math functions for vs3, CEdge, stereo calculations, ...
 * 
 * TODO Doxygen description
 **/


#ifndef Z_MATH_HH
#define Z_MATH_HH

#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <stdexcept>

// namespace Z					/// TODO Sollte eigentlich eigenen Namespace haben!
// {

const unsigned UNDEF_ID = UINT_MAX;

const int START     = 0;
const int END       = 1;
const int MID       = 2;
const int ONE_THIRD = 3;
const int TWO_THIRD = 4;
const int STOP      = 15;

inline int OtherEnd(int end)
{
  return end ^ 0x1;
}

// same and opposite sense
const int SAME = START;
const int OPP  = END;

const int LEFT  = 0;
const int RIGHT = 1;

// inner and outer ends of junctions
const int INNER = 0;
const int OUTER = 1;

inline int OtherSide(int side)
{
  return side ^ 0x1;
}

inline int Other(int i)
{
  return i ^ 0x1;
}


extern void InitMath();
extern unsigned HammingDistance(unsigned a, unsigned b);
extern bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2);		/// TODO ARI: Clip Line in Math.xx ??
extern bool ClipLine(int xmin, int ymin, int xmax, int ymax, double *x1, double *y1, double *x2, double *y2);
extern double Fact(int n);
extern double LogFact(int n);
extern double LogBinCoef(int n, int k) throw (std::runtime_error);
extern double LogBinDist(int l, int k, double p);
extern double LogBinomialPDF(int l, int k, double p);
extern double BinomialPDF(int l, int k, double p);
extern double LogBinomialCDF(int l, int k, double p);
extern double BinomialCDF(int l, int k, double p);
extern double LogBinomialCDF_tail(int l, int k, double p);
extern double BinomialCDF_tail(int l, int k, double p);
extern double Significance(int m, int k, int l, double p);
extern double PoissonPDF(int k, double alpha);
extern double PoissonCDF(int k, double alpha);
extern int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);
extern double timespec_diff(struct timespec *x, struct timespec *y);


extern int RandInt();
extern bool IsZero(double d);                  /// Returns true if the value is near zero (+/- epsilon)
extern bool IsEqual(double a, double b);       /// Returns true if the values are equal (+/- epsilon)
extern bool IsReasonable(double d);            /// Return whether value is a 'reasonable' number

template <class Num>
extern Num Sqr(Num x);                         /// Square of given number

template <class Num>
extern Num Max(Num a, Num b);

template <class Num>
extern Num Min(Num a, Num b);

template <class Num>
extern int Sign(Num x);

template <class T>
extern inline void Swap(T &a, T &b);

template <class Num>
inline bool Between(Num x, Num l, Num u);

template <class Num>
inline bool BetweenEq(Num x, Num l, Num u);

/// Scale angle to [0..2pi[
extern double ScaleAngle_0_2pi(double a);
/// Scale angle to [-pi..pi[
extern double ScaleAngle_mpi_pi(double a);
/// Scale angle to [0..pi[
extern double ScaleAngle_0_pi(double a);
/// Difference of two angles b - a. The result is scaled to [-pi..pi[
extern double DiffAngle_mpi_pi(double b, double a);
/// Difference of two angles b - a. The result is scaled to [0..2pi[
extern double DiffAngle_0_2pi(double b, double a);
/// Angles between (undirected) lines. The result is scaled to [0..pi/2[
extern double AngleBetweenLines(double b, double a);
/// Scale an integer angle to [0..8[
extern int ScaleIntAngle_0_8(int a);

extern unsigned BinaryToGray(unsigned b) throw (std::runtime_error);
extern int CityblockDistance(int x1, int y1, int x2, int y2);
extern double BivariateGaussianPDF(double mx, double my, double s, double x, double y);

extern double SphereVolume(double r);
extern double CircleArea(double r);
extern float DistSqr(float *desc1, float *desc2, unsigned size);

// Returns a pseudo random number in [0.0, 1.0]
static float FRand();
// Exponential PDF
static float ExpPdf(float lambda);
// Return an int <= max with probability according to expnential PDF
static int ExpSelect(int max);

// }

#include "Math.ic"

#endif

