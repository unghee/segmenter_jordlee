/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

#include "SurfaceModeling.hh"

#define USE_UNKNOWN_PATCHES     // Use also unexplained patches for merging (non-planar patches)
#define TRY_MERGING             // Try merging of patches
#define MS_PARALLEL             // parallel plane/NURBS calculation and merging
// #define NO_MS_CHECK             // do not check model merging (not suggested!)

namespace surface {

using namespace std;

template<typename T1,typename T2>
inline T1 Dot3(const T1 v1[3], const T2 v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

template<typename T1,typename T2, typename T3>
inline void Mul3(const T1 v[3], T2 s, T3 r[3])
{
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

/********************** SurfaceModeling ************************
 * Constructor/Destructor
 */
SurfaceModeling::SurfaceModeling(Parameter p, bool _dbgPts3D, bool _dbgTxt3D) :
  dbgPts3D(_dbgPts3D), dbgTxt3D(_dbgTxt3D), haveIntr(false), haveExtr(false), param(p)
{}

SurfaceModeling::~SurfaceModeling()
{
}

/************************** PRIVATE ************************/

/**
 * ComputeSavings
 */
double SurfaceModeling::ComputeSavings(int numParams, std::vector<double> &probs)
{
  double savings = probs.size() - param.kappa1 * (double) numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  savings -= param.kappa2 * err;

  return (savings > 0 ? savings : 0);
}

/**
 * ComputeSavingsNormalized
 */
double SurfaceModeling::ComputeSavingsNormalized(int numParams, std::vector<double> &probs, double norm)
{
  norm = 1. / norm;
  double savings = norm * (double) probs.size() - param.kappa1 * (double) numParams;
  double err = 0.;

  for (unsigned i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);

  savings -= norm * param.kappa2 * err;

  return (savings > 0 ? savings : 0);
}

/**
 * AddToQueue
 */
void SurfaceModeling::AddToQueue(const std::vector<unsigned> &nbs, std::vector<unsigned> &queue)
{
  for (unsigned i = 0; i < nbs.size(); i++) {
    queue.push_back(nbs[i]);
  }
}

/**
 * FitNurbs
 */
void SurfaceModeling::FitNurbs(SurfaceModel &surface)
{
  pcl::PointIndices::Ptr points(new pcl::PointIndices());
  points->indices = surface.indices;

  cv::Ptr<pcl::on_nurbs::SequentialFitter> nurbsFitter;
  nurbsFitter = new pcl::on_nurbs::SequentialFitter(param.nurbsParams);
  if (haveIntr && haveExtr) {
    nurbsFitter->setProjectionMatrix(camIntr, camExtr);
  } else {
    printf("[SurfaceModeling::compute] Warning, projection matrix not set!\n");
  }
  
  nurbsFitter->setInputCloud(cloud);
  nurbsFitter->setInterior(points);
  nurbsFitter->compute();
  nurbsFitter->getInteriorError(surface.error);
  surface.nurbs = nurbsFitter->getNurbs();
  nurbsFitter->getInteriorNormals(surface.normals);
  nurbsFitter->getInteriorParams(surface.nurbs_params);

  // check orientation of normals and calculate probabilities
  Eigen::Vector3d n0(0., 0., 1.);
  for (unsigned i = 0; i < surface.normals.size(); i++) {
    Eigen::Vector3d &n = surface.normals[i];

    if (Dot3(&n[0], &n0[0]) > 0)
        Mul3(&n[0], -1, &n[0]);
  }
  ComputePointProbs(surface.error, surface.probs);
}

/**
 * ModelSelection with omp parallel
 */
void SurfaceModeling::ModelSelectionParallel()
{
//  #pragma omp parallel for    // TODO This does not work always
  for (unsigned i = 0; i < models.size(); i++) {
    if (models[i]->selected && !models[i]->used  && models[i]->type == pcl::SACMODEL_PLANE) {
      if((int)models[i]->indices.size() < param.planePointsFixation) {
        SurfaceModel::Ptr model;
        model.reset(new SurfaceModel());
        model->indices = models[i]->indices;
        model->type = models[i]->type;
        model->idx = models.size();
        model->selected = true;
        model->used = false;
        model->neighbors3D = models[i]->neighbors3D;
          
        if(model->indices.size() > 3)
          FitNurbs(*model);
      
        models[i]->savings = ComputeSavingsNormalized(12, models[i]->probs, models[i]->indices.size());
        model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * 3, model->probs, models[i]->indices.size());

#ifdef DEBUG
        cout << "Savings plane/NURBS id=" << i << ": " << models[i]->savings << "/" << model->savings;
#endif

        if (model->savings > models[i]->savings) { // => create NURBS
          model->type = MODEL_NURBS;
          model->savings = models[i]->savings;
          models[i] = model;
        }
#ifdef DEBUG
        cout << "-> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
#endif
      }
    }
  }
  
#ifdef TRY_MERGING
#ifdef DEBUG
        cout << "-> Try merging of surface models:" << endl;
#endif

  std::vector<merge> merge_pairs;
  
#pragma omp parallel for shared(merge_pairs) //default(none)
  for (unsigned i = 0; i < models.size(); i++) {
    if (models[i]->selected && !models[i]->used) {
      if(models[i]->indices.size() < param.planePointsFixation) {
        for(unsigned j =0; j<models[i]->neighbors3D.size(); j++) {
          unsigned idx = models[i]->neighbors3D[j];
          if(idx > i) {
            SurfaceModel::Ptr mergedModel;
            mergedModel.reset(new SurfaceModel());
            (*mergedModel) = *models[i];
            models[idx]->AddTo(*mergedModel);

            if(mergedModel->indices.size() > 3)
              FitNurbs(*mergedModel);
            mergedModel->savings = ComputeSavingsNormalized(
                mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * 3, mergedModel->probs, mergedModel->indices.size());
            models[i]->savings = ComputeSavingsNormalized(
                (models[i]->type == MODEL_NURBS ? models[i]->nurbs.m_cv_count[0] * models[i]->nurbs.m_cv_count[1] * 3 : 12), models[i]->probs, mergedModel->indices.size());
            models[idx]->savings = ComputeSavingsNormalized((models[idx]->type == MODEL_NURBS ? models[idx]->nurbs.m_cv_count[0] * models[idx]->nurbs.m_cv_count[1] * 3 : 12),
                models[idx]->probs, mergedModel->indices.size());

            if (mergedModel->savings > (models[i]->savings + models[idx]->savings)) {
              merge m;
              m.id_1 = i;
              m.id_2 = idx;
              m.savings = mergedModel->savings;
              
#pragma omp critical 
              {
                merge_pairs.push_back(m);
#ifdef DEBUG
  cout << "-> Merge candidates: " << m.id_1 << "-" << m.id_2 << endl;
#endif
              }
            }
          }
        }
      }
    }
  }
  
  // merge the surfaces from the best to the weakest connection
  std::sort(merge_pairs.begin(), merge_pairs.end(), CmpSavings);
  
  for(unsigned i=0; i<merge_pairs.size(); i++) {
    if(merge_pairs[i].id_1 != merge_pairs[i].id_2) {
      
#ifdef NO_MS_CHECK
      models[merge_pairs[i].id_2]->AddTo(*models[merge_pairs[i].id_1]);
      FitNurbs(*models[merge_pairs[i].id_1]);
      models[merge_pairs[i].id_2]->selected = false;
      
#ifdef DEBUG        
      printf(" => MERGED: %u-%u\n", merge_pairs[i].id_1, merge_pairs[i].id_2);
#endif
        
      for(unsigned j=i+1; j < merge_pairs.size(); j++) {
        if(merge_pairs[j].id_1 == merge_pairs[i].id_2)
          merge_pairs[j].id_1 = merge_pairs[i].id_1;
        if(merge_pairs[j].id_2 == merge_pairs[i].id_2)
          merge_pairs[j].id_2 = merge_pairs[i].id_1;
      }
#else

      SurfaceModel::Ptr mergedModel;
      mergedModel.reset(new SurfaceModel());
      (*mergedModel) = *models[merge_pairs[i].id_1];
      models[merge_pairs[i].id_2]->AddTo(*mergedModel);
      if(mergedModel->indices.size() > 3)
        FitNurbs(*mergedModel);
      mergedModel->savings = ComputeSavingsNormalized(
                mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * 3, mergedModel->probs, mergedModel->indices.size());
      models[merge_pairs[i].id_1]->savings = ComputeSavingsNormalized(
                (models[merge_pairs[i].id_1]->type == MODEL_NURBS ? models[merge_pairs[i].id_1]->nurbs.m_cv_count[0] * models[merge_pairs[i].id_1]->nurbs.m_cv_count[1] * 3 : 12), 
                 models[merge_pairs[i].id_1]->probs, mergedModel->indices.size());
      models[merge_pairs[i].id_2]->savings = ComputeSavingsNormalized(
                (models[merge_pairs[i].id_2]->type == MODEL_NURBS ? models[merge_pairs[i].id_2]->nurbs.m_cv_count[0] * models[merge_pairs[i].id_2]->nurbs.m_cv_count[1] * 3 : 12),
                models[merge_pairs[i].id_2]->probs, mergedModel->indices.size());
      
#ifdef DEBUG        
      printf("Try merging %u and %u (%1.5f > %1.5f)\n", merge_pairs[i].id_1, merge_pairs[i].id_2, mergedModel->savings, models[merge_pairs[i].id_1]->savings + models[merge_pairs[i].id_2]->savings);
#endif

      if (mergedModel->savings > (models[merge_pairs[i].id_1]->savings + models[merge_pairs[i].id_2]->savings)) {
        mergedModel->savings = ComputeSavingsNormalized((mergedModel->type == MODEL_NURBS ? mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * 3 : 12), mergedModel->probs, mergedModel->indices.size());

#ifdef DEBUG        
        printf(" => MERGED: %u-%u\n", merge_pairs[i].id_1, merge_pairs[i].id_2);
#endif
        mergedModel->type = MODEL_NURBS;
        models[merge_pairs[i].id_1] = mergedModel;
        models[merge_pairs[i].id_2]->selected = false;

        for(unsigned j=i+1; j < merge_pairs.size(); j++) {
          if(merge_pairs[j].id_1 == merge_pairs[i].id_2)
            merge_pairs[j].id_1 = merge_pairs[i].id_1;
          if(merge_pairs[j].id_2 == merge_pairs[i].id_2)
            merge_pairs[j].id_2 = merge_pairs[i].id_1;
        }
      }
#endif
    }
  }
#endif
}


/**
 * Old ModelSelection (non-parallel version)
 */
void SurfaceModeling::ModelSelection()
{
  SurfaceModel::Ptr model, mergedModel;
  vector<unsigned> queue;

  for (unsigned i = 0; i < models.size(); i++) {
    // compare single model
    if (models[i]->selected && !models[i]->used) {
      model.reset(new SurfaceModel());
      model->indices = models[i]->indices;

      FitNurbs(*model);

      models[i]->savings = ComputeSavingsNormalized(12, models[i]->probs, models[i]->indices.size());
      model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * 3, model->probs,
          models[i]->indices.size());
#ifdef DEBUG
      cout << "Savings plane/NURBS id=" << i << ": " << models[i]->savings << "/" << model->savings;
#endif

      if (model->savings > models[i]->savings) {
        models[i]->selected = false;

        model->type = MODEL_NURBS;
        model->idx = models.size();
        model->selected = true;
        model->neighbors3D = models[i]->neighbors3D;

        models.push_back(model);
      } 
      else 
        model = models[i];
      
#ifdef DEBUG
      cout << "-> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
#endif

      // try merge;
#ifdef TRY_MERGING
      unsigned idx;
      if((model->type != MODEL_NURBS &&  (int)model->indices.size() < param.planePointsFixation) || model->type == MODEL_NURBS)
      {
        queue = model->neighbors3D;
        while (queue.size() > 0) {
          idx = queue.back();
          if (models[idx]->selected && !models[idx]->used) {
            mergedModel.reset(new SurfaceModel());
            (*mergedModel) = *model;
            models[idx]->AddTo(*mergedModel);

            FitNurbs(*mergedModel);

            mergedModel->savings = ComputeSavingsNormalized(
                mergedModel->nurbs.m_cv_count[0] * mergedModel->nurbs.m_cv_count[1] * 3, mergedModel->probs,
                mergedModel->indices.size());
            model->savings = ComputeSavingsNormalized(
                (model->type == MODEL_NURBS ? model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * 3 : 12), model->probs,
                mergedModel->indices.size());
            models[idx]->savings = ComputeSavingsNormalized(
                (models[idx]->type == MODEL_NURBS ? models[idx]->nurbs.m_cv_count[0] * models[idx]->nurbs.m_cv_count[1] * 3 : 12),
                models[idx]->probs, mergedModel->indices.size());
#ifdef DEBUG
            cout << "Try merging ids " << i << "-" << idx << ": " << model->savings << "+" << models[idx]->savings << "="
                << model->savings + models[idx]->savings << " / " << mergedModel->savings;
#endif

            if (mergedModel->savings > (model->savings + models[idx]->savings)) {
#ifdef DEBUG
              cout << " MERGED" << endl;
              if(model->type != MODEL_NURBS)
                printf("              ===> MERGE PLANE TO NURBS (size: %lu)!!!\n", model->indices.size());
#endif
              models[idx]->selected = false;
              model->selected = false;

              mergedModel->type = MODEL_NURBS;
              mergedModel->idx = models.size();
              mergedModel->selected = true;

              models.push_back(mergedModel);

              queue.pop_back();
              AddToQueue(models[idx]->neighbors3D, queue);
              model = mergedModel;
              model->used = true;
              models[idx]->used = true;
            } 
            else {
#ifdef DEBUG
              cout << endl;
#endif
              queue.pop_back();
            }
          }
          else
            queue.pop_back();
        }
        models[i]->used = true;
      }
#endif
    }
  }
}


/**
 * ComputePointProbs
 */
void SurfaceModeling::ComputePointProbs(std::vector<double> &errs, std::vector<double> &probs)
{
  probs.resize(errs.size());
  for (unsigned i = 0; i < errs.size(); i++)
    probs[i] = exp(-((errs[i]*errs[i]) * invSqrSigmaError));
}



/**
 * ComputePointError
 */
void SurfaceModeling::ComputePointError(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                             const std::vector<SurfaceModel::Ptr> &planes,
                                             std::vector< std::vector<double> > &error)
{
  error.clear();
  for (unsigned j=0; j<planes.size(); j++) {
    SurfaceModel &plane = *planes[j];

    if(plane.type == pcl::SACMODEL_PLANE) {
      float a, b, c, d;
      std::vector<double> s_error;
      a=plane.coeffs[0], b=plane.coeffs[1], c=plane.coeffs[2], d=plane.coeffs[3];

      for (unsigned i=0; i<plane.indices.size(); i++)
        s_error.push_back(Plane::ImpPointDist(a,b,c,d, &cloud.points[plane.indices[i]].x));
      error.push_back(s_error);
    }
    else {
      std::vector<double> s_error;
      for(unsigned i=0; i<plane.indices.size(); i++)
        s_error.push_back(0.00);  // TODO arbitrary error, if we do not have the model
      error.push_back(s_error);
    }
  }
}

/**
 * SetPointNormals
 */
void SurfaceModeling::SetPointNormals(std::vector<float> &_coeffs,
    std::vector<int> &_indices, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals)
{
  Eigen::Vector3d n0(0., 0., 1.);
  normals.resize(_indices.size());

  if (Dot3(&_coeffs[0], &n0[0]) > 0) {
    Mul3(&_coeffs[0], -1, &_coeffs[0]);
    _coeffs[3] = -_coeffs[3];
  }

  for (unsigned i = 0; i < _indices.size(); i++) {
    Eigen::Vector3d &n = normals[i];
    n[0] = _coeffs[0];
    n[1] = _coeffs[1];
    n[2] = _coeffs[2];
  }
}

/**
 * InitDataStructure
 */
void SurfaceModeling::InitDataStructure()
{
  SurfaceModel::Ptr ptrModel;
  models.clear();
  for (unsigned i = 0; i < modelTypes.size(); i++) {
    ptrModel.reset(new SurfaceModel());
    models.push_back(ptrModel);
    SurfaceModel &model = *models.back();

    model.idx = i;
    model.type = modelTypes[i];
    model.coeffs = coeffs[i]; 
    model.indices = indices[i];
    model.error = error[i];
    model.selected = true;
    ComputePointProbs(model.error, model.probs);
    SetPointNormals(model.coeffs, model.indices, model.normals);
  }
}

/**
 * ComputeMean 
 */
cv::Point SurfaceModeling::ComputeMean(std::vector<int> &_indices)
{
  cv::Point mean(0., 0.);

  if (_indices.size() == 0)
    return mean;

  for (unsigned i = 0; i < _indices.size(); i++) {
    mean += cv::Point(X(_indices[i]), Y(_indices[i]));
  }

  mean.x /= _indices.size();
  mean.y /= _indices.size();

  return mean;
}

/**
 * computeNeighbors 
 */
void SurfaceModeling::computeNeighbors()
{
  patches = cv::Mat_<cv::Vec3b>(cloud->height, cloud->width);
  patches.setTo(0);
  for(unsigned i=0; i<models.size(); i++) {
    for(unsigned j=0; j<models[i]->indices.size(); j++) {
      int row = models[i]->indices[j] / cloud->width;
      int col = models[i]->indices[j] % cloud->width;
      patches.at<cv::Vec3b>(row, col)[0] = i+1;  /// plane 1,2,...,n
    }
  }
  nr_patches = models.size();

  neighbors2D.resize(0);
  neighbors3D.resize(0);

  bool nbgh_matrix3D[nr_patches+1][nr_patches+1];
  bool nbgh_matrix2D[nr_patches+1][nr_patches+1];
  for(unsigned i=0; i<nr_patches+1; i++)
    for(unsigned j=0; j<nr_patches+1; j++) {
      nbgh_matrix3D[i][j] = false;
      nbgh_matrix2D[i][j] = false;
    }
  
  #pragma omp parallel for
  for(int row=1; row<patches.rows; row++) {
    for(int col=1; col<patches.cols; col++) {
      if(patches.at<cv::Vec3b>(row, col)[0] != 0) {
        if(patches.at<cv::Vec3b>(row, col)[0] != patches.at<cv::Vec3b>(row-1, col)[0]) {
          if(patches.at<cv::Vec3b>(row-1, col)[0] != 0) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = (row-1)*cloud->width+col;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
              nbgh_matrix3D[patches.at<cv::Vec3b>(row-1, col)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
              nbgh_matrix3D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row-1, col)[0]] = true;
            }
            nbgh_matrix2D[patches.at<cv::Vec3b>(row-1, col)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
            nbgh_matrix2D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row-1, col)[0]] = true;
          }
        }
        if(patches.at<cv::Vec3b>(row, col)[0] != patches.at<cv::Vec3b>(row, col-1)[0]) {
          if(patches.at<cv::Vec3b>(row, col-1)[0] != 0) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = row*cloud->width+col-1;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
              nbgh_matrix3D[patches.at<cv::Vec3b>(row, col-1)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
              nbgh_matrix3D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row, col-1)[0]] = true;
            }
            nbgh_matrix2D[patches.at<cv::Vec3b>(row, col-1)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
            nbgh_matrix2D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row, col-1)[0]] = true;
          }
        }
        if(patches.at<cv::Vec3b>(row, col)[0] != patches.at<cv::Vec3b>(row-1, col-1)[0]) {
          if(patches.at<cv::Vec3b>(row-1, col-1)[0] != 0) {
            int pos_0 = row*cloud->width+col;
            int pos_1 = (row-1)*cloud->width+col-1;
            double dis = fabs(cloud->points[pos_0].z - cloud->points[pos_1].z);
            if( dis < param.z_max) {
              nbgh_matrix3D[patches.at<cv::Vec3b>(row-1, col-1)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
              nbgh_matrix3D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row-1, col-1)[0]] = true;
            }
            nbgh_matrix2D[patches.at<cv::Vec3b>(row-1, col-1)[0]][patches.at<cv::Vec3b>(row, col)[0]] = true;
            nbgh_matrix2D[patches.at<cv::Vec3b>(row, col)[0]][patches.at<cv::Vec3b>(row-1, col-1)[0]] = true;
          }
        }
      }
    }
  }
  
  for(unsigned i=1; i<nr_patches+1; i++) {
    std::vector<unsigned> neighbor;
    for(unsigned j=1; j<nr_patches+1; j++) {
      if(nbgh_matrix3D[i][j])
        neighbor.push_back(j-1);
    }
    neighbors3D.push_back(neighbor);
  }
  for(unsigned i=1; i<nr_patches+1; i++) {
    std::vector<unsigned> neighbor;
    for(unsigned j=1; j<nr_patches+1; j++) {
      if(nbgh_matrix2D[i][j])
        neighbor.push_back(j-1);
    }
    neighbors2D.push_back(neighbor);
  }
  
  for(unsigned i=0; i< neighbors3D.size(); i++)
    models[i]->neighbors3D = neighbors3D[i];
  for(unsigned i=0; i< neighbors2D.size(); i++)
    models[i]->neighbors2D = neighbors2D[i];
}

/**
 * SetBoundary
 */
void SurfaceModeling::ComputeBoundary()
{
  pclA::ContourDetection cont(pclA::ContourDetection::Parameter(true, false, 5));
  cont.setInputCloud(cloud);

  for (unsigned i = 0; i < models.size(); i++) {
    SurfaceModel &model = *models[i];
    cont.compute(model.indices, model.contour);
  }
}


/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void SurfaceModeling::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
}

/**
 * setInputPlanes
 */
void SurfaceModeling::setInputPatches(const std::vector<SurfaceModel::Ptr> _patches)
{
  if(cloud.get() == 0)
    printf("[SurfaceModeling::setInputPatches] Error: Set input cloud first.\n");
  
  org_patches = _patches;
  modelTypes.clear();
  coeffs.clear();
  indices.clear();
  error.clear();
  for(unsigned i=0; i<_patches.size(); i++) {
#ifndef USE_UNKNOWN_PATCHES
    if(_patches[i]->type == pcl::SACMODEL_PLANE)
#endif
    {
      modelTypes.push_back(_patches[i]->type);
      coeffs.push_back(_patches[i]->coeffs);
      indices.push_back(_patches[i]->indices);
    }
  }
  ComputePointError(*cloud, _patches, error);
}

/**
 * setIntrinsic
 */
void SurfaceModeling::setIntrinsic(double fx, double fy, double cx, double cy)
{
  camIntr = Eigen::Matrix4d::Zero();
  camIntr(0, 0) = fx;
  camIntr(1, 1) = fy;
  camIntr(0, 2) = cx;
  camIntr(1, 2) = cy;
  camIntr(2, 2) = 1.0;
  haveIntr = true;
}

/**
 * setExtrinsic
 */
void SurfaceModeling::setExtrinsic(Eigen::Matrix4d &pose)
{
  camExtr = pose;
  haveExtr = true;
}

/**
 * Operate
 */
void SurfaceModeling::compute()
{
#ifdef DEBUG
  cout << "************************ MODEL A NEW FRAME *********************************" << endl;
#endif

  invSqrSigmaError = 1. / (param.sigmaError * param.sigmaError);
  InitDataStructure();
  computeNeighbors();

#ifdef MS_PARALLEL
  ModelSelectionParallel();
#else
  ModelSelection();
#endif

  ComputeBoundary();
  computeNeighbors();

#ifdef DEBUG
//   if (!dbgWin.empty() && (dbgPts3D || dbgTxt3D)) {
//     dbgWin->Clear();
//     cv::Mat_<cv::Vec4f> cvCloud;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPatch(new pcl::PointCloud<pcl::PointXYZRGB>());
//     unsigned cntNurbs = 0, cntPlanes = 0;
//     for (unsigned i = 0; i < models.size(); i++) {
//       if (models[i]->selected) {
//         if (dbgPts3D) {
//           pcl::copyPointCloud(*cloud, models[i]->indices, *cloudPatch);
//           float col = surface::GetRandomColor();
//           for (unsigned j = 0; j < cloudPatch->points.size(); j++)
//             cloudPatch->points[j].rgb = col;
//           pclA::ConvertPCLCloud2CvMat(cloudPatch, cvCloud);
//           dbgWin->AddPointCloud(cvCloud);
//         }
//         //draw labels
//         if (dbgTxt3D) {
//           cv::Point pt = ComputeMean(models[i]->indices);
//           pcl::PointXYZRGB &pt3 = (*cloud)(pt.x, pt.y);
//           string label = P::toString(models[i]->idx, 0);
//           if (models[i]->type == MODEL_NURBS) {
//             label = label + "N";
//             cntNurbs++;
//           } else {
//             cntPlanes++;
//             label = label + "P";
//           }
//           dbgWin->AddLabel3D(label, 12, pt3.x, pt3.y, pt3.z);
//         }
//       }
//     }
//     cout << "cntPlanes=" << cntPlanes << ", cntNurbs=" << cntNurbs << " -> selected=" << cntPlanes + cntNurbs << endl;
//   }
#endif
}

/**
 * getSurfaceModels
 */
void SurfaceModeling::getSurfaceModels(std::vector<SurfaceModel::Ptr> &_models, bool addUnknownData)
{
  _models.clear();
  for (unsigned i = 0; i < models.size(); i++) {
    if (models[i]->selected) {
      
      if(addUnknownData) {
        models[i]->idx = _models.size();
        _models.push_back(models[i]);
      }
      else {
        if(models[i]->type == pcl::SACMODEL_PLANE || models[i]->type == MODEL_NURBS) {
          models[i]->idx = _models.size();
          _models.push_back(models[i]);
        }
      }
    }
  }
}


#ifdef DEBUG
/**
 * SetDebugWin
 */
void SurfaceModeling::SetDebugWin(cv::Ptr<TomGine::tgTomGineThread> &win)
{
  dbgWin = win;
  dbgWin->SetRotationCenter(0.0, 0.0, 1.0);
}
#endif

} //-- THE END --

