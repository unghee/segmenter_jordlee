/**
 * @file tgNurbsSurface
 * @author Thomas Moerwald
 * @date May 2011
 * @version 0.1
 * @brief NURBS surface representation and its GPU (GLSL) implementation "nurbscurve.vert", "coxdeboor.c"
 */

#ifndef _TG_NURBS_SURFACE_H_
#define _TG_NURBS_SURFACE_H_

#include <vector>

#include "tgMathlib.h"
#include "tgTexture.h"
#include "tgShader.h"
#include "tgShapeCreator.h"
#include "tgRenderModel.h"
#include "tgSingleton.h"

namespace TomGine {

/** @brief NURBS surface representation for one continuous patch. */
struct tgNurbsSurfacePatch
{
  std::vector<float> knotsU, knotsV; ///< Knot vectors.
  std::vector<vec4> cps; ///< Control point vector. A control point exists of a 3D vector and the weight as 4th entry.
  unsigned ncpsU, ncpsV; ///< Number of control points.
  unsigned orderU, orderV; ///< Order of NURBS.
  unsigned resU, resV; ///< Resolution of surface grid.
  bool sync; ///< Indicating memory synchronization with GPU.
  /** @brief Create empty NURBS surface. */
  tgNurbsSurfacePatch()
  {
    ncpsU = 0;
    ncpsV = 0;
    orderU = 2;
    orderV = 2;
    resU = 16;
    resV = 16;
    sync = false;
  }
  /** @brief Print NURBS data to console. */
  void Print()
  {
    printf("tgNurbsSurfacePatch: \n");
    printf("orderU: %d, orderV: %d, resU: %d, resV: %d\n", orderU, orderV, resU, resV);
    printf("Knot vector U: ");
    for( unsigned i = 0; i < knotsU.size(); i++ )
      printf("%f ", knotsU[i]);
    printf("\n");
    printf("Knot vector V: ");
    for( unsigned i = 0; i < knotsV.size(); i++ )
      printf("%f ", knotsV[i]);
    printf("\n");
    printf("Control points:\n");
    for( unsigned i = 0; i < cps.size(); i++ )
      printf("[%f %f %f %f]\n", cps[i].x, cps[i].y, cps[i].z, cps[i].w);
  }
};

/** @brief NURBS surface representation on the GPU using OpenGL/GLSL ("nurbssurface.vert", "coxdeboor.c"). */
class tgNurbsSurface
{
private:
  tgTexture1D texKnotsU, texKnotsV; ///< GPU storage for knot vectors.
  tgTexture2D texCP; ///< GPU storage for control points RGBA, where RGB is the xyz position in 3D space and A is the weight.
  tgRenderModel m_model; ///< Quadrangle mesh discretising NURBS surface.
  std::vector<tgLine> m_boundary;
  GLUnurbsObj *m_glunurb; ///< GLU representation of NURBS (redundant information for testing)

  tgShader *m_NurbsSurfaceShader; ///< GLSL Shader for NURBS curves.

  tgNurbsSurfacePatch nurbsData; ///< Storage for the NURBS data in RAM (for CPU usage).

  void SetShader();

public:
  /** @brief empty. */
  tgNurbsSurface(TomGine::tgShader *nurbsSurfaceShader);

  /** @brief Load data for drawing NURBS surface and NURBS data to GPU. Remesh surface. */
  tgNurbsSurface(const tgNurbsSurfacePatch &data, TomGine::tgShader *nurbsSurfaceShader);

  static tgShader* LoadShader();

  /** @brief Load NURBS surface data to GPU. */
  void Set(const tgNurbsSurfacePatch &data);

  /** @brief Change resolution (discretisation) of mesh */
  void Remesh(unsigned resU, unsigned resV);

  /** @brief Set control point at index position i, j
   *  @param i,j	Index of control point. Index corresponding to U and V direction of surface.
   *  @param cpv	Control point vector; values of the new control points. */
  void SetCP(unsigned i, unsigned j, const vec4 &cpv);

  /** @brief Set control point at index position i.
   *  @param i	Index of control point. Index of control point vector (direct access).
   *  @param cpv	Control point vector; values of the new control points. */
  void SetCP(unsigned i, const vec4 &cpv);

  /** @brief Get control point at index position i, j. Index corresponding to U and V direction of surface. */
  vec4 GetCP(unsigned i, unsigned j);

  /** @brief Get control point at index position i. Index of control point vector (direct access).  */
  vec4 GetCP(unsigned i);

  /** @brief Draws surface as point grid */
  void DrawVertices();

  /** @brief Draw surface as quad mesh */
  void DrawFaces();

  /** @brief Draw surface as quad mesh
   *  @params r,g,b	color for quad mesh */
  void DrawFaces(float r, float g, float b);

  /** @brief Draw boundary of surface */
  void DrawBoundary(float r, float g, float b, float linewidth = 1.0f);

  /** @brief Draw control points of surface */
  void DrawCPs();

};

} // namespace TomGine

#endif //_TG_NURBS_SURFACE_H_
