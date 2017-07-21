/**
 * @file Ellipse.hh
 * @author Zillich, Richtsfeld
 * @date 2006, Dec. 2009, June 2010
 * @version 0.1
 * @brief Gestalt feature Ellipse
 */

#ifndef Z_ELLIPSE_HH
#define Z_ELLIPSE_HH

#include "Edgel.hh"
#include "Gestalt.hh"
#include "ConvexArcGroup.hh"
#include "VisionCore.hh"

namespace Z
{

class EJunction;

/**
 * @brief Gestalt class ellipse.
 */
class Ellipse : public Gestalt
{
private:
  class ContourPoint
  {
  public:
    int x, y;
    double err;
    unsigned id;
    double dir;
    bool supports;
    ContourPoint() {};
    ContourPoint(int xi, int yi, double erri)
      : x(xi), y(yi), err(erri), id(UNDEF_ID), dir(0.), supports(false) {}
  };
	
  Array<ContourPoint> contour;
  double a2, b2, a4, b4;  // a^2 etc., stored for Distance()

  void CalculateSupport();
  void CalculateSignificance();
  Vector2 TransformToEllipse(const Vector2 &p);
  Vector2 TransformFromEllipse(const Vector2 &p);
  double DistanceCentAxPar(const Vector2 &p);
  bool GradientMatches(const Edgel &e);
  bool GradientMatches(const Vector2 &p, double dir);
  bool GradientMatchesCentAxPar(const Vector2 &p, double dir);
  Vector2 Tangent(const Vector2 &p);
  Vector2 TangentCentAxPar(const Vector2 &p);
  Vector2 Normal(const Vector2 &p);
  Vector2 NormalCentAxPar(const Vector2 &p);
  void OptimalNeighbour(int x, int y, int a, int b, int &x_o, int &y_o,
      double &d_o);
  void BuildContourString();
  void FillContourString();

public:
  ConvexArcGroup *group;			///< group the ellipse was constucted from
  double x, y, a, b, phi;			///< ellipse parameters
  double support;							///< relative support
  double abs_support;					///< absulute support
  double fit_error;						///< mean fit error
  double area;								///< area of the ellipse
  Vector2 center;							///< center of the ellipse
  Vector2 vertex[2];					///< vertex [LEFT/RIGHT]
	Vector2 dir;								///< direction between vertices
	Array<EJunction*> ejcts[2];	///< ellipse junctions of this ellipse [LEFT/RIGHT]


  Ellipse(VisionCore *vc, ConvexArcGroup *grp_in, double x_in, double y_in,
      double a_in, double b_in, double phi_in);
  virtual void Draw(int detail = 0);
  void DrawVotes();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  double Circumference();
  double Area();
  double Distance(const Vector2 &p);
  double Distance(const Edgel &e);
};

inline Array<Gestalt*>& Ellipses(VisionCore *core)
{
  return core->Gestalts(Gestalt::ELLIPSE);
}
inline Ellipse* Ellipses(VisionCore *core, unsigned id)
{
  return (Ellipse*)core->Gestalts(Gestalt::ELLIPSE, id);
}
inline unsigned NumEllipses(VisionCore *core)
{
  return core->NumGestalts(Gestalt::ELLIPSE);
}

extern bool FitEllipse(Array<Arc*> &arcs, unsigned l, unsigned u,
    double &x, double &y, double &a, double &b, double &phi);
extern bool FitEllipse(Array<Arc*> &arcs,
    double &x, double &y, double &a, double &b, double &phi);
extern double EllipseCircumference(double a, double b);
extern double EllipseArea(double a, double b);
extern double EllipseSupport(Array<Arc*> &arcs, unsigned l, unsigned u,
    double x, double y, double a, double b, double phi, double &fit_error,
    double *abs_sup = 0);
extern double EllipseSupport(Array<Arc*> &arcs,
    double x, double y, double a, double b, double phi, double &fit_error,
    double *abs_Sup);
extern double EllipseCurvature(double a, double b, double x, double y);

}

#endif

