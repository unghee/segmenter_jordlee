/**
 * $Id$
 */

#include "CreateMeshModel.hh"

// #ifndef DEBUG
// #define DEBUG
// #endif

using namespace objectmodeling;

using namespace std;

/********************** CreateMeshModel ************************
 * Constructor/Destructor
 */
CreateMeshModel::CreateMeshModel (Parameter p) :
  param (p)
{
}

CreateMeshModel::~CreateMeshModel ()
{
}

/************************** PRIVATE ************************/

/**
 * ProjectPointsToPlane
 */
void
CreateMeshModel::ProjectPointsToPlane (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<int> &indices,
                                       const std::vector<float> &coeffs,
                                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &points)
{
  if (coeffs.size () != 4)
    throw std::runtime_error ("[CreateMeshModel::ProjectPointsToPlane] Wrong plane parameters!");

  Eigen::Vector4d mc (coeffs[0], coeffs[1], coeffs[2], 0);
  Eigen::Vector4d p (0, 0, 0, 1), pp;
  float dist;

  mc.normalize ();

  Eigen::Vector4d tmp_mc = mc;
  tmp_mc[3] = coeffs[3];

  points.resize (indices.size ());
  for (unsigned i = 0; i < indices.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.points[indices[i]];
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;

    dist = tmp_mc.dot (p);

    pp = p - mc * dist;

    Eigen::Vector3d &ptOut = points[i];
    ptOut[0] = pp[0];
    ptOut[1] = pp[1];
    ptOut[2] = pp[2];
  }
}

void
CreateMeshModel::ProjectPointsToPlane (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<int> &indices,
                                       const std::vector<float> &coeffs,
                                       std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &points)
{
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
  ProjectPointsToPlane (cloud, indices, coeffs, points3D);

  points.clear ();

  Eigen::Matrix3d R, Rt;
  Eigen::Vector3d t;
  objectmodeling::Triangulation::getPlaneBasis (coeffs, R, t);
  Rt = R.transpose ();

  for (size_t i = 0; i < points3D.size (); i++)
  {
    Eigen::Vector3d p = Rt * (points3D[i] - t);
    points.push_back (Eigen::Vector2d (p (0), p (1)));
  }
}

/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void
CreateMeshModel::setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
}

/**
 * Operate
 */
void
CreateMeshModel::compute (std::vector<surface::SurfaceModel::Ptr> &models)
{
  for (size_t i = 0; i < models.size (); i++)
  {
    surface::SurfaceModel::Ptr model = models[i];
    if (model->type == MODEL_NURBS)
    {
      objectmodeling::Triangulation::TriangulatNurbs (model->nurbs, model->mesh, model->nurbs_params,
                                                      param.max_concavity);
    }
    else if (model->type == pcl::SACMODEL_PLANE)
    {
      std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > points;
      ProjectPointsToPlane (*cloud, model->indices, model->coeffs, points);
      objectmodeling::Triangulation::DelauneyTriangulation (points, model->mesh, param.max_concavity);

      Eigen::Matrix3d R;
      Eigen::Vector3d t;
      objectmodeling::Triangulation::getPlaneBasis (model->coeffs, R, t);

      for (unsigned n = 0; n < model->mesh.m_vertices.size (); n++)
      {
        TomGine::tgVertex &v = model->mesh.m_vertices[n];
        Eigen::Vector3d p (v.pos.x, v.pos.y, v.pos.z);
        p = R * p + t;
        v.pos = TomGine::vec3 (p (0), p (1), p (2));

        v.normal.x = model->coeffs[0];
        v.normal.y = model->coeffs[1];
        v.normal.z = model->coeffs[2];
        v.normal.normalize ();
      }
    }
  }
}

void
CreateMeshModel::computeUnTrimmed (surface::SurfaceModel::Ptr surface)
{
  if (surface->nurbs.KnotCount (0) > 1 || surface->nurbs.KnotCount (1) > 1)
  {
    objectmodeling::Triangulation::convertNurbs2tgModel (surface->nurbs, surface->mesh, param.resolution, false);
  }
  else if (surface->type == pcl::SACMODEL_PLANE)
  {
    pcl::on_nurbs::vector_vec2d points;
    ProjectPointsToPlane (*cloud, surface->indices, surface->coeffs, points);
    Eigen::Vector2d a, b;
    Triangulation::computeBoundingBox (points, a, b);
    Eigen::Vector4d bb (a (0), b (0), a (1), b (1));
    objectmodeling::Triangulation::convertPlane2tgModel (surface->coeffs, surface->mesh, bb, param.resolution);
  }
}

void
CreateMeshModel::computeUntrimmed (surface::View &view)
{
  for (size_t i = 0; i < view.surfaces.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = view.surfaces[i];
    computeUnTrimmed (surface);
  }
}

void
CreateMeshModel::computeTrimmed (surface::SurfaceModel::Ptr surface, unsigned resolution)
{
  if (surface->curves_param.empty ())
  {
    printf ("[CreateMeshModel::computeTrimmed] Warning, curves not computed (params missing)\n");
    return;
  }

  ON_NurbsCurve &curve = surface->curves_param[0];

  if (surface->nurbs.KnotCount (0) <= 1 || surface->nurbs.KnotCount (1) <= 1)
  {
    printf ("[CreateMeshModel::computeTrimmed] Error, ON_NurbsSurface not valid\n");
    return;
  }
  objectmodeling::Triangulation::convertTrimmedSurface2tgModel (surface->nurbs, curve, surface->mesh, resolution,
                                                                resolution, false);
}

void
CreateMeshModel::computeTrimmed (surface::View &view)
{
  for (size_t i = 0; i < view.surfaces.size (); i++)
  {
    surface::SurfaceModel::Ptr surface = view.surfaces[i];

    computeTrimmed (surface, param.resolution);

  }
}

