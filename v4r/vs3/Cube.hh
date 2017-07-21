/**
 * @file Cube.hh
 * @author Richtsfeld Andreas
 * @date March 2010
 * @version 0.1
 * @brief Header file of Gestalt Cube.
 **/

#ifndef Z_CUBE_HH
#define Z_CUBE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"
#include "FlapAri.hh"
#include "Rectangle.hh"

namespace Z
{

/**
 * @brief Class of Gestalt Cube.
 */
class Cube : public Gestalt
{
public:
	FlapAri *flap[3];								///< The 3 flaps, building the cube.
	Rectangle *rectangle[3];				///< The 3 rectangles, building a cube
	Vector2 cornerPoint[8];					///< The 8 corner points of the cube
																	// Zero is in the center, 0-6 clockwise r0[1,2,3], r1[3,4,5], r2[5,6,1]
																	// cornerPoint[7] is the hidden corner point of the cube.

	bool plausible;									///< If flaps are ordered correct, cube is plausible.
	double parallelity;							///< value for the parallity of the 3x4 parallel cube-edges to calc. significance
	Vector2 center;									///< center of the cube-area
	double radius;									///< circumscribed radius
//	double maxRatio;								///< TODO maximum ratio between height, depth and width of the cube

	Cube(VisionCore *vc, FlapAri *f[3]);
	bool PlausibilityCheck(FlapAri *f[3]);
	void GetCornerPoints();
  void CalculateProperties();
	double CheckParallelity();
  void CalculateSignificance();
  //bool CheckW2HRatio();

  bool IsInside(unsigned cube);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Cubes(VisionCore *core)
{
  return core->Gestalts(Gestalt::CUBE);
}
inline Cube* Cubes(VisionCore *core, unsigned id)
{
  return (Cube*)core->Gestalts(Gestalt::CUBE, id);
}
inline unsigned NumCubes(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CUBE);
}

}

#endif
