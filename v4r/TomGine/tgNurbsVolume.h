/**
* @file tgNurbsVolume
* @author Thomas Moerwald
* @date May 2011
* @version 0.1
* @brief NURBS volume representation and its GPU (GLSL) implementation "nurbsvolume.vert", "coxdeboor.c"
*/

#ifndef _TG_NURBS_VOLUME_H_
#define _TG_NURBS_VOLUME_H_

#include <vector>

#include "tgMathlib.h"
#include "tgTexture.h"
#include "tgShader.h"
#include "tgShapeCreator.h"
#include "tgRenderModel.h"

namespace TomGine{

/** @brief NURBS volume representation for one continuous patch. */
struct tgNurbsVolumePatch{
	std::vector<float> knotsU, knotsV, knotsW;	///< Knot vectors.
    std::vector<vec4> cps;						///< Control point vector. A control point exists of a 3D vector and the weight as 4th entry.
    unsigned ncpsU, ncpsV, ncpsW;				///< Number of control points
    unsigned orderU, orderV, orderW;			///< Order of NURBS
    unsigned resU, resV, resW;					///< Resolution of mesh grid.
    bool sync;									///< Indicating memory synchronization with GPU.
    /** @brief Create empty NURBS volume. */
    tgNurbsVolumePatch(){
    	ncpsU = 0; ncpsV = 0; ncpsW = 0;
    	orderU = 2; orderV = 2; orderW = 0;
    	resU = 16; resV = 16; resW = 16;
    	sync = false;
    }
    /** @brief Print NURBS data to console. */
    void Print(){
    	printf("tgNurbsVolumePatch: \n");
    	printf("orderU: %d, orderV: %d, orderW: %d, resU: %d, resV: %d, resW: %d\n",
    			orderU, orderV, orderW, resU, resV, resW);
    	printf("Knot vector U: ");
    	for(unsigned i=0; i<knotsU.size(); i++)
    		printf("%f ", knotsU[i]);
    	printf("\n");
    	printf("Knot vector V: ");
    	for(unsigned i=0; i<knotsV.size(); i++)
    		printf("%f ", knotsV[i]);
    	printf("\n");
    	printf("Knot vector W: ");
    	for(unsigned i=0; i<knotsW.size(); i++)
    		printf("%f ", knotsW[i]);
    	printf("\n");
    	printf("Control points:\n");
    	for(unsigned i=0; i<cps.size(); i++)
    		printf("[%f %f %f %f]\n", cps[i].x, cps[i].y, cps[i].z, cps[i].w);
    }
};

/** @brief NURBS surface representation on the GPU using OpenGL/GLSL ("nurbssurface.vert", "coxdeboor.c"). */
class tgNurbsVolume
{
private:
	tgTexture1D texKnotsU, texKnotsV, texKnotsW;	///< GPU storage for knot vectors.
	tgTexture3D texCP;								///< GPU storage for control points RGBA, where RGB is the xyz position in 3D space and A is the weight.
    tgShader *shNurbsSurface;						///< GLSL Shader for NURBS volumes.
    tgRenderModel m_model;							///< Boundary quadrangle mesh discretising NURBS volume.

    tgNurbsVolumePatch nurbsData;					///< Storage for the NURBS data in RAM (for CPU usage).

    tgNurbsVolume( const tgNurbsVolume &v);
    tgNurbsVolume& operator=(const tgNurbsVolume &v);

public:
	/** @brief Load shader for drawing NURBS volume and NURBS data to GPU. Remesh volume. */
	tgNurbsVolume( const tgNurbsVolumePatch &data );

	/** @brief Load shader for drawing NURBS volume. */
	tgNurbsVolume();

	/** @brief Destroys shader. */
	~tgNurbsVolume();

	/** @brief Load NURBS surface data to GPU. */
	void Set( const tgNurbsVolumePatch &data );

	/** @brief Change resolution (discretisation) of mesh */
	void Remesh(unsigned resU, unsigned resV, unsigned resW);

	/** @brief Set control point at index position i,j,k
	 *  @param i,j,k	Index of control point. Index corresponding to U,V,W direction of volume.
	 *  @param cpv		Control point vector; values of the new control points. */
	void SetCP(unsigned i, unsigned j, unsigned k, const vec4 &cpv);

	/** @brief Set control point at index position i.
	 *  @param i	Index of control point. Index of control point vector (direct access).
	 *  @param cpv	Control point vector; values of the new control points. */
	void SetCP(unsigned i, const vec4 &cpv);

	/** @brief Get control point at index position i,j,k. Index corresponding to U,V,W direction of volume. */
	vec4 GetCP(unsigned i, unsigned j, unsigned k);

	/** @brief Get control point at index position i. Index of control point vector (direct access).  */
	vec4 GetCP(unsigned i);

	/** @brief Get number of control points (size of control point vector).  */
	inline unsigned GetCPSize(){ return nurbsData.cps.size(); }

	/** @brief Draws surface as point grid */
	void DrawVertices();

	/** @brief Draw surface as quad mesh (only at boundaries). */
	void DrawFaces();

	/** @brief Draw control points of volume */
	void DrawCPs();

	/** @brief Initialize NURBS to form a cuboid. */
	void CreateBox();

};

} // namespace TomGine

#endif //_TG_NURBS_VOLUME_H_
