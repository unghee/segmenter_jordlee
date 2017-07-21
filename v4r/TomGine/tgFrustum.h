 /**
 * @file tgFrustum.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief View frustum of a camera.
 */

#ifndef TG_FRUSTUM
#define TG_FRUSTUM

#include "headers.h"

#include "tgMathlib.h"

namespace TomGine{

/** @brief View frustum, defined by 6 planes. (near, far, left, right, bottom, top) */
class tgFrustum{
    
private:
	float frustum[6][4];	///< The plane parameters
	mat4 m_intrinsic;		///< intrinsic camera matrix
	mat4 m_extrinsic;		///< extrinsic camera matrix

	/** @brief Extracts the view frustum from the currently set projection- and modelview-matrix. */
	void ExtractFrustum();

	friend class tgCamera;

public:

	/** @brief Check whether a point lies in the frustum.
	 *  @return true	Point lies in the frustum.
	 *  @return false	Point lies outside the frustum.
	 *  @param x,y,z	The point to check. */
	bool PointInFrustum( float x, float y, float z );

	/** @brief Check whether a shpere lies completely in the frustum.
	 *  @return true	Sphere lies completely in the frustum.
	 *  @return false	Point lies completely outside the frustum.
	 *  @param x,y,z	The center of the sphere.
	 *  @param radius	The readius of the sphere. */
	bool SphereInFrustum( float x, float y, float z, float radius );

	/** @brief Draws the view frustum using GL_LINES. */
	void DrawFrustum();

};

} // namespace TomGine

#endif
