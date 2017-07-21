 /**
 * @file tgNurbsCurve.h
 * @author Thomas Mörwald
 * @date May 2011
 * @version 0.1
 * @brief NURBS curve representation and its GPU (GLSL) implementation "nurbscurve.vert", "coxdeboor.c"
 */

#ifndef _TG_NURBS_CURVE_H_
#define _TG_NURBS_CURVE_H_

#include <vector>

#include "tgMathlib.h"
#include "tgTexture.h"
#include "tgShader.h"
#include "tgModel.h"

namespace TomGine{

/** @brief NURBS curve representation for one continuous patch. */
struct tgNurbsCurvePatch{
	std::vector<float> knotsU;	///< Knot vector.
    std::vector<vec4> cps;		///< Control point vector. A control point exists of a 3D vector and the weight as 4th entry.
    unsigned ncpsU;				///< Number of control points.
    unsigned orderU;			///< Order of NURBS.
    unsigned resU;				///< Resolution of curve grid.
    bool sync;					///< Indicating memory synchronization with GPU.
    /** @brief Create empty NURBS curve. */
    tgNurbsCurvePatch(){
    	ncpsU = 0;
    	orderU = 2;
    	resU = 16;
    	sync = false;
    }
    /** @brief Print NURBS data to console. */
    void Print(){
    	printf("tgNurbsCurvePatch: \n");
    	printf("orderU: %d, resU: %d\n", orderU, resU);
    	printf("Knot vector U: ");
    	for(unsigned i=0; i<knotsU.size(); i++)
    		printf("%f ", knotsU[i]);
    	printf("\n");
    	printf("Control points:\n");
    	for(unsigned i=0; i<cps.size(); i++)
    		printf("[%f %f %f %f]\n", cps[i].x, cps[i].y, cps[i].z, cps[i].w);

    }
};

/** @brief NURBS curve representation on the GPU using OpenGL/GLSL ("nurbscurve.vert", "coxdeboor.c"). */
class tgNurbsCurve
{
private:
	tgTexture1D texKnots;			///< GPU storage for knot vector.
	tgTexture1D texCP;				///< GPU storage for control points RGBA, where RGB is the xyz position in 3D space and A is the weight.
    tgShader *shNurbsCurve;			///< GLSL Shader for NURBS curves
    std::vector<tgVertex> m_points;	///< Points discretising the NURBS curve

    tgNurbsCurvePatch nurbsData;	///< Storage for the NURBS data in RAM (for CPU usage).

public:
    /** @brief Load shader for drawing NURBS curves and NURBS data to GPU. Remesh curve with data::resU. */
	tgNurbsCurve( const tgNurbsCurvePatch &data );

	/** @brief Load shader for drawing NURBS curves. */
	tgNurbsCurve();

	/** @brief Destroys shader. */
	~tgNurbsCurve();

	/** @brief Load NURBS curve data to GPU. */
	void Set( const tgNurbsCurvePatch &data );

	static tgShader* LoadShader();

	/** @brief Change resolution (discretisation) of curve. */
	void Remesh(unsigned res);

	/** @brief Set control point at index position i.
	 *  @param i	Index of control point.
	 *  @param cpv	Control point vector; values of the new control points. */
	void SetCP(unsigned i, const vec4 &cpv);

	/** @brief Get control point at index position i. */
	vec4 GetCP(unsigned i);

	/** @brief Draws surface as point grid. */
	void DrawVertices();

	/** @brief Draw surface as line strip. */
	void DrawLines();

	/** @brief Draw control points of surface. */
	void DrawCPs();

};

} // namespace TomGine

#endif //_TG_NURBS_SURFACE_H_
