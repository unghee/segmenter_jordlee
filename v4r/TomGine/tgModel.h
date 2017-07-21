 /**
 * @file tgModel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining a model for rendering.
 */

#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>

#include "tgMathlib.h"

namespace TomGine{

/** @brief Vertex representing a directed point in 3D space with texture coordinates. See glVertex, glNormal, glTexCoord in OpenGL spec. */
struct tgVertex{
	vec3 pos;				///< 3D position of vertex
	vec3 normal;			///< Normal vector of vertex
	vec2 texCoord;			///< Texture coordinate of vertex
};
/** @brief Face of a surface built by indexed vertices. See glBegin(GL_QUAD), glBegin(GL_TRIANGLE) in OpenGL spec. */
struct tgFace{
	std::vector<unsigned> v;///< List of vertex-indices
	vec3 normal;			///< Normal vector of face
};
/** @brief Line defined by a starting and end point in 3D space. See glBegin(GL_LINE) in OpenGL spec. */
struct tgLine{
	tgLine(){};
	tgLine(vec3 s, vec3 e) : start(s), end(e){};
	vec3 start;	///< Start point of the line.
	vec3 end;	///< End point of line.
};
/** @brief Ray define by starting point and direction. */
struct tgRay{
	tgRay(){}
	tgRay(vec3 s, vec3 e) : start(s), dir(e){}
	vec3 start;	///< Start point of the ray.
	vec3 dir;	///< Vector/direction of the ray (view ray).
};
/** @brief A point in 3D with a color assigned. See glBegin(GL_POINTS), glColor in OpenGL spec. */
struct tgColorPoint{
	vec3 pos;					///< Position of the point.
	unsigned char color[3];		///< Color of the point in RGB [0..255].
};
/** @brief Bounding sphere of the model defined by the center and radius. */
struct BoundingSphere{
	vec3 center;	///< Center point of the sphere.
	float radius;	///< Radius of the sphere.
	BoundingSphere() : center(0.0f,0.0f,0.0f), radius(-1.0f){}
};
/** @brief Rectangle in 2D defined by a point (x,y), width and height. */
struct tgRect2D{
	float x,y,w,h;
	tgRect2D(float x, float y, float w, float h){ this->x=x; this->y=y; this->w=w; this->h=h;}
	tgRect2D(){ this->x=0.0f; this->y=0.0f; this->w=1.0f; this->h=1.0f;}
};

struct tgRect2Di{
	int x,y, w,h;
	tgRect2Di(int x, int y, int w, int h){ this->x=x; this->y=y; this->w=w; this->h=h;}
	tgRect2Di(){ this->x=0; this->y=0; this->w=2; this->h=2;}
};

/** @brief Geometric representation of various primitives (triangles, quadrangles, lines, points, ...) */
class tgModel{
public:
  std::vector<tgVertex>	m_vertices;				///< list of vertices
  std::vector<tgFace>		m_faces;				///< list of faces
  std::vector<tgLine>		m_lines;				///< list of lines
  std::vector<vec3>		m_points;				///< list of points
  std::vector<tgColorPoint> m_colorpoints;		///< list of colored points
  BoundingSphere  		m_bs;					///< bounding sphere

  float m_point_size;
  float m_line_width;
  vec3 m_line_color;
  vec3 m_point_color;

  tgModel() : m_point_size(1.0f), m_line_width(1.0f), m_line_color(1.0, 0.0, 0.0), m_point_color(1.0, 0.0, 0.0){}

  //	virtual tgModel& operator+=(const tgModel& m);
  virtual void Merge(const tgModel &m);

  /** @brief Save data access to vertices
   *  @param i	index of vertex in list m_vertices */
  tgVertex	getVertex(unsigned int i){ if(i<m_vertices.size() && i>=0) return m_vertices[i]; else return tgVertex();}

  /** @brief Save data access to faces
   *  @param i	index of face in list m_faces */
  tgFace		getFace(unsigned int i){ if(i<m_faces.size() && i>=0) return m_faces[i]; else return tgFace();}

  /** @brief Draw all data in model. */
  virtual void Draw();

  /** @brief Draw vertices as points. */
  virtual void DrawVertices() const;

  /** @brief Draws triangles and quadrangles given by m_faces. */
  virtual void DrawFaces() const;

  /** @brief Draws lines given by m_lines all with the color given. */
  virtual void DrawLines() const;

  /** @brief Draws points given by m_points all with the color given. */
  virtual void DrawPoints() const;

	/** @brief Draws colored points, given by m_colorpoints. */
  virtual void DrawColorPoints() const;

  /** @brief Draws normals of vertices in m_faces.
   *  @param length The length of the line representing the normal. */
  virtual void DrawNormals(float length = 1.0) const;

  /** @brief Checks for consistence of faces. */
  virtual void CheckFaces() const;

  /** @brief Compute normals of vertices using cross product of faces. */
  virtual void ComputeNormals();

  /** @brief Compute normals of vertices of m_faces, m_polygons, m_quadstrips. */
  virtual void ComputeFaceNormals();

  /** @brief Compute bounding sphere which contains all vertices.*/
  virtual void ComputeBoundingSphere();

  /** @brief Clears data of model (m_vertices and m_faces). */
  virtual void Clear();

  /** @brief Prints infos of model to console. */
  virtual void Print() const;

};

} // namespace TomGine

#endif
