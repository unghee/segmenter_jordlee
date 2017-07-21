#include <stdio.h>

char *StdGeomDef[]={
  "Line Startx Starty Endx Endy",
  "DashLine Startx Starty Endx Endy",
  "DotLine Startx Starty Endx Endy",
  "Arc  Centerx Centery Startx Starty Endx Endy",
  "Circle Centerx Centery Startx Starty",
  "Ellipse Centerx Centery Major Minor Angle",
  "Point x y",
  "PointBox x y",
  "PointStar x y",
  "PointCross x y",
  "PointDot x y",
  "PointSet REP x y",
  "PolyLine REP x y",
  "PolyDashLine REP x y",
  "PolyDotLine REP x y",
  "FilledPoly REP x y",
  NULL
};
int StdGeomNum = sizeof(StdGeomDef)/sizeof(char *) - 1;


