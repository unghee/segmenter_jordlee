
//pcl
//#include <iostream>
//#include <pcl/point_types.h>

#include "Segmenter1.h"
//#include <pcl/segmentation/region_growing.h>

/* --------------- Segmenter --------------- */

//Segmenter::Segmenter segmenter;
namespace segment
{

Segmenter::Segmenter(std::string _db, std::string _rgbd, std::string _model, bool _live)
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = _db;
  rgbd_filename = _rgbd;
  model_path = _model;
  data_live = _live;
  startIdx = 0;
  endIdx = 65;
  useStructuralLevel = true;
  useAssemblyLevel = false;
}
Segmenter::~Segmenter()
{
  delete kinect;
  delete surfModeling;
  delete patchRelations;
  delete svm1st;
  delete svm2nd;
  delete graphCut;
  delete resultSaver;
}

void Segmenter::init()
{

  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  std::string svmStructuralModel = model_path + "PP-Trainingsset.txt.scaled.model";
  std::string svmStructuralScaling = model_path + "param.txt";
  std::string svmAssemblyModel = model_path + "PP2-Trainingsset.txt.scaled.model";
  std::string svmAssemblyScaling = model_path + "param2.txt";


  surface::ClusterNormalsToPlanes::Parameter param;
  param.adaptive = true;         // use adaptive thresholds
  clusterNormals = new surface::ClusterNormalsToPlanes(param);

  // init nurbsfitting & model-selection

  pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16;
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.planePointsFixation = 8000;       // 8000
  sfmParams.z_max = 0.01;
  surfModeling = new surface::SurfaceModeling(sfmParams);
  surfModeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling->setExtrinsic(pose);

  // init patch relation class
  patchRelations = new surface::PatchRelations();
  patchRelations->setStructuralLevel(useStructuralLevel);
  patchRelations->setAssemblyLevel(useAssemblyLevel);

  // init svm-predictor
  svm1st = new svm::SVMPredictorSingle(svmStructuralModel.c_str());
  svm2nd = new svm::SVMPredictorSingle(svmAssemblyModel.c_str());
  svm1st->setScaling(true, svmStructuralScaling.c_str());
  svm2nd->setScaling(true, svmAssemblyScaling.c_str());

  // init graph cutter
  graphCut = new gc::GraphCut();

  // save results to sfv file
  resultSaver = new surface::SaveFileSequence();
  resultSaver->InitFileSequence("result_%1d.sfv", 0, 1000);

}

void Segmenter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);
  pc_=cloud;

}


std::vector<pcl::PointIndices>
Segmenter::processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
 // pclA::FilterZ(pcl_cloud, z_min, z_max);

  // calcuate normals
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter param;
  param.adaptive = true;
  pclA::ZAdaptiveNormals nor(param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);

  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);

  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(surfaces, false);
  printf("surfmodeling\n");
  std::vector<surface::Relation> relation_vector;
  patchRelations->setInputCloud(pcl_cloud);
  printf("setinputcloud\n");
  patchRelations->setSurfaceModels(surfaces);
  printf("setSurfaceModels\n");
  patchRelations->setOptimalPatchModels(true);
  printf("OptimalPatchModel\n");
  patchRelations->computeSegmentRelations();
  printf("computeSegmentRelation\n");
  patchRelations->getRelations(relation_vector);
  printf("getrelation\n");
  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st->getResult(relation_vector[i].type,
                                                        relation_vector[i].rel_value,
                                                        relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd->getResult(relation_vector[i].type,
                                                        relation_vector[i].rel_value,
                                                        relation_vector[i].rel_probability);
  }
//githubtest
  graphCutGroups.clear();
  graphCut->init(surfaces.size(), relation_vector);
  graphCut->process();
  graphCut->getResults(surfaces.size(), graphCutGroups);
//  graphCut->printResults();

  std::vector<pcl::PointIndices> results;
  results.resize(graphCutGroups.size());
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
      for(unsigned k=0; k<surfaces[graphCutGroups[i][j]]->indices.size(); k++)
        results[i].indices.push_back(surfaces[graphCutGroups[i][j]]->indices[k]);
  return results;
}


//  void Segmenter::run(std::string _rgbd_filename,std::string _model_path, int _startIdx, int _endIdx)
  void Segmenter::run(std::string _model_path)
  {
  bool processed = false;
  database_path = "";
  model_path = _model_path;
 // rgbd_filename = _rgbd_filename;



    printf("init.\n");
  init();

    //point from pointcloud
    //make error message
    string point_cloud_topic("/head_camera/depth_registered/points");


    ros::NodeHandle nh;
    ros::Subscriber sub;

    std::cout<< "subscribing pointcloud "<< std::endl;
    sub = nh.subscribe(point_cloud_topic, 1, &segment::Segmenter::pointCloudCallback,this);
    ros::spin();


    std::cout<<" processing pointcloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZRGB>);
    // ######################## Setup TomGine ########################
    int width = 640;
    int height = 480;
    surface::View view;

    TomGine::tgTomGineThread dbgWin(width, height, "TomGine Render Engine");
    cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Vec3d rotCenter(0, 0, 1.0);

    cv::Mat intrinsic;
    intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    view.intrinsic = Eigen::Matrix3d::Zero();
    intrinsic.at<double> (0, 0) = intrinsic.at<double> (1, 1) = view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
    intrinsic.at<double> (0, 2) = view.intrinsic(0, 2) = 320;
    intrinsic.at<double> (1, 2) = view.intrinsic(1, 2) = 240;
    intrinsic.at<double> (2, 2) = view.intrinsic(2, 2) = 1.;

    dbgWin.SetClearColor(0.5, 0.5, 0.5);
    dbgWin.SetCoordinateFrame();
    dbgWin.SetCamera(intrinsic);
    dbgWin.SetCamera(R, t);
    dbgWin.SetRotationCenter(rotCenter);
    dbgWin.Update();
    cv::Mat_<cv::Vec3b> kImage = cv::Mat_<cv::Vec3b>::zeros(480, 640);
 //   cv::Mat_<cv::Vec3b> image;

 //   cv::namedWindow( "Debug image",cv::WINDOW_AUTOSIZE);


    //loading pointcloud from pcd
   // pcl::io::loadPCDFile (rgbd_filename, *cloud_input);
    pcl::copyPointCloud(*pc_, *cloud_input);
    pclA::ConvertPCLCloud2Image(cloud_input, kImage);
    cv::imshow("Debug image", kImage);
    dbgWin.SetImage(kImage);
    dbgWin.Update();
    pcl::io::savePCDFileASCII("/home/fetch/catkin_ws/src/segmenter_jordlee/testsegment.pcd",*cloud_input);
    std::vector<pcl::PointIndices> label_indices;
  //  pcl::copyPointCloud(*(processPointCloudV(cloud_input)), *cloud_output);

    label_indices = processPointCloudV(cloud_input);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud_output;
    std::cout<<"before"<<std::endl;
    for(int i =0;i<label_indices.size();i++)
    {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_input, label_indices[i], *(cloud_temp));
    cloud_output.push_back(cloud_temp);

    }
    std::cout<<"after"<<std::endl;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    rgbVis(cloud_output,label_indices);


    printf("[Segmenter::run] Done.\n");
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer>

  Segmenter::rgbVis (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud, std::vector<pcl::PointIndices> label)
  {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    std::vector<string> idname;
    idname.reserve(label.size());
    viewer->setBackgroundColor (0, 0, 0);

    for(int i=0;i<label.size();i++)
    {

      string s = boost::lexical_cast<string>(i);
      idname.push_back(s);
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> rgb(cloud[i]);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud[i], rgb, idname[i]);

    }
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (viewer);
  }


}

void printUsage(char *av)
{
    printf("Usage: %s [options] \n"
               " Options:\n"
               "   [-h] ... show this help.\n"
               "   [-f rgbd_filename] ... specify rgbd-image path and filename\n"
               "   [-m model_path] ... specify model path\n"
               "   [-idx start end] ... start and end index of files\n"
               "   [-l] ... live image from kinect\n"
               "   [-as usage] ... use assembly level: 0/1\n", av);
    std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -m model/ -idx 0 10" << std::endl;
}


int main(int argc, char **argv)
{

  std::string rgbd_filename = "points2/test%1d.pcd";
 // std::string model_path = "model/";
  std::string model_path = "/home/fetch/catkin_ws/src/segmenter_jordlee/model/";

  /*
  for(int i=1; i<argc; i++) {
    if(strcmp (argv[i], "-h") == 0) {
      printUsage(argv[0]);
      exit(0);
    }
    if(i+1 < argc) {
      if(strcmp (argv[i], "-f") == 0)
        rgbd_filename = argv[i+1];
      if(strcmp (argv[i], "-m") == 0)
        model_path = argv[i+1];

      }


    else
      printUsage(argv[0]);

  }
*/
  //Receiving pointclouds

  ros::init(argc, argv, "segmenter");
  segment::Segmenter seg;
  //seg.setMinMaxDepth(0.0, 1.5);
//  seg.run(rgbd_filename, model_path, startIdx, endIdx);
  seg.run(model_path);


  //ros::spin();


}
