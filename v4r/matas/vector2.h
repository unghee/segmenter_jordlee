#ifndef V2_H
#define V2_H

#include <limits.h>

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif

typedef struct {float x,y;} t_V2;

#define UNDEF_V2 1.234E4
#define ZERO_V2  1E-5 

/* useful convesions  */
#ifndef V2PI
#define V2PI         3.14159265
#endif

#ifndef Deg2Rad
#define Deg2Rad(a) ((a)*V2PI/180)
#define Rad2Deg(a) ((a)*180/V2PI)
#endif


t_V2 ConsV2( float x, float y);
t_V2 * DupV2(t_V2 in);

int  IsEqualV2(t_V2 v1, t_V2 v2);

t_V2 DiffV2( t_V2 v1, t_V2 v2);
t_V2 AddV2( t_V2 v1, t_V2 v2);
t_V2 ScalV2( t_V2 v1, float ratio);

float DotPV2( t_V2 v1, t_V2 v2);
float CrossV2( t_V2 v1, t_V2 v2);

t_V2 NormV2( t_V2 v);
t_V2 NormClockV2( t_V2 v);
t_V2 NormAClockV2( t_V2 v);

float MagnV2( t_V2 v1 );
float MagnSqrV2( t_V2 v1 );

float AngleV2(t_V2 v1, t_V2 v2);
float PolarAngleV2(t_V2 v);
float DistV2( t_V2 point1, t_V2 point2);
float DistVtoLine(t_V2 Point, t_V2 PonLine1, t_V2 PonLine2);
float SignDistVtoLine(t_V2 Point, t_V2 PonLine1, t_V2 PonLine2);

t_V2 IntSecV2(t_V2 s1, t_V2 e1, t_V2 s2,t_V2 e2,float *p1,float *p2);
t_V2 CollIntSecV2(t_V2 s1, t_V2 e1, t_V2 s2,t_V2 e2, int *r1, int *r2);
float LineAngleV2(t_V2 v1, t_V2 v2);

/*
t_V2 ProjV2 ( t_V2 Point, t_V2 PonLine, float LineAngle);
*/
t_V2 ProjV2( t_V2 Point, t_V2 PonLine, t_V2 LineDir);
t_V2 ProjV2_v1( t_V2 Point, t_V2 PonLine, t_V2 LineDir);

t_V2 RotV2 (t_V2 v, float rads);
/*----------------------------------------------------------------------*/
typedef struct {int x,y;} t_V2i;
#define UNDEF_V2I INT_MAX

t_V2i ConsV2i( float x, float y);
t_V2i * DupV2i(t_V2i in);

int  IsEqualV2i(t_V2i v1, t_V2i v2);

t_V2i DiffV2i( t_V2i v1, t_V2i v2);
t_V2i AddV2i( t_V2i v1, t_V2i v2);
t_V2i ScalV2i( t_V2i v1, int ratio);
t_V2i DivV2i( t_V2i v1, int fact);

int DotPV2i( t_V2i v1, t_V2i v2);
int CrossV2i( t_V2i v1, t_V2i v2);

t_V2i NormV2i( t_V2i v);
t_V2i NormClockV2i( t_V2i v);
t_V2i NormAClockV2i( t_V2i v);

int MagnSqrV2i( t_V2i v1 );
t_V2i IntSecV2i(t_V2i p1, t_V2i a1, t_V2i p2, t_V2i a2);
int DistSqrV2i(t_V2i v1, t_V2i v2);

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif
