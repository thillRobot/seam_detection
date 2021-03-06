

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

int
main (int argc, char *argv[])
{
  parseCommandLine (argc, argv);

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  //
  //  Set up resolution invariance
  //
  if (use_cloud_resolution_)
  {
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);

  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}

















  //int N_cor=100;
  //EigenCor cor_src_pts, cor_tgt_pts;
  //register_cloud_teaser(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_params, cor_src_pts, cor_tgt_pts);
  //register_cloud_FPFHteaser(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_params, cor_src_pts, cor_tgt_pts);

// A work in progress - maybe later
/*
// This function REGISTER_CLOUD finds the transform between two pointclouds using PCL::registration (ICP from scratch - but why?) 
// parts of this function work 
void register_cloud_PCL(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double params[], EigenCor &cor_src_pts, EigenCor &cor_tgt_pts)
{
 
  int Ns = cloud_source.size();
  int Nt = cloud_target.size();
  int P = 50; //number to print
  int M = -1; //number of matches
  std::cout <<"Beginning Correspondence Estimation with PCL"<< std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> estimator;
  estimator.setInputCloud (source);
  estimator.setInputTarget (target);
  //estimator.determineReciprocalCorrespondences (*correspondences); // use reciprocal correspondences
  estimator.determineCorrespondences (*correspondences);             

  // check for correct order and number of matches

  //if (int (correspondences->size ()) == nr_original_correspondences)
  //{
  
  std::cout <<"Correspondence Estimation Complete"<< std::endl;
  std::cout <<"Index, Index Query, Index Match"<< std::endl;
  for (int i = 0; i < Ns; ++i){
    if (i<P)
      std::cout <<i<<","<<(*correspondences)[i].index_query<<","<<(*correspondences)[i].index_match<< std::endl;
    if ((*correspondences)[i].index_match == -1)
      M=i;
  }
  std::cout << M << " matches found"<< std::endl ;
  
  pcl::PointCloud<pcl::PointNormal>::Ptr source_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*source, *source_normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr target_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*target, *target_normals);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est_src;
  norm_est_src.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est_src.setKSearch (10);
  norm_est_src.setInputCloud (source_normals);
  norm_est_src.compute (*source_normals);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est_tgt;
  norm_est_tgt.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est_tgt.setKSearch (10);
  norm_est_tgt.setInputCloud (target_normals);
  norm_est_tgt.compute (*target_normals);

  pcl::registration::CorrespondenceRejectorSurfaceNormal  corr_rej_surf_norm;
  corr_rej_surf_norm.initializeDataContainer <pcl::PointXYZ, pcl::PointNormal> ();
  corr_rej_surf_norm.setInputSource <pcl::PointXYZ> (source);
  corr_rej_surf_norm.setInputTarget <pcl::PointXYZ> (target);
  corr_rej_surf_norm.setInputNormals <pcl::PointXYZ, pcl::PointNormal> (source_normals);
  corr_rej_surf_norm.setTargetNormals <pcl::PointXYZ, pcl::PointNormal> (target_normals);

  boost::shared_ptr<pcl::Correspondences>  correspondences_result_rej_surf_norm (new pcl::Correspondences);
  corr_rej_surf_norm.setInputCorrespondences (correspondences);
  corr_rej_surf_norm.setThreshold (0.5);

  corr_rej_surf_norm.getCorrespondences (*correspondences_result_rej_surf_norm);

  std::cout <<"Surface Normal Correspondence Rejector Complete"<< std::endl;
  std::cout <<"Index, Index Query, Index Match"<< std::endl;

  int M_rsn=-1;
  //for (int i = 0; i < P; ++i)
  //  std::cout <<i<<","<<(*correspondences_result_rej_surf_norm)[i].index_query
  //               <<","<<(*correspondences_result_rej_surf_norm)[i].index_match<< std::endl;

                  //std::cout <<"Index, Index Query, Index Match"<< std::endl;
  for (int i = 0; i < Ns; ++i){
    if (i<P)
      std::cout <<i<<","<<(*correspondences)[i].index_query<<","<<(*correspondences)[i].index_match<< std::endl;
    if ((*correspondences_result_rej_surf_norm)[i].index_match == -1)
      M_rsn=i;
  }
  std::cout << M << " matches found out of " << Ns << " source points and " <<Nt<<" target points" << std::endl ;

  std::cout <<"CONVERTING CORRESPONDENCE POINTCLOUDS TO EIGEN" << std::endl;
  //int N = cloud_source.size();
  //int N = 50;
  int Nc = 1000;  // number of correspondences to process
  int src_idx,tgt_idx;
  // Convert the point cloud to Eigen
  EigenCor cor_src(3, Ns);
  EigenCor cor_tgt(3, Ns);
  for (size_t i = 0; i < Nc; ++i) {
    //src_idx=(*correspondences)[i].index_query; // correspondences before rejection
    //tgt_idx=(*correspondences)[i].index_match;
    src_idx=(*correspondences_result_rej_surf_norm)[i].index_query; // correspondences after rejection
    tgt_idx=(*correspondences_result_rej_surf_norm)[i].index_match;
    cor_src.col(i) << cloud_source[src_idx].x, cloud_source[src_idx].y, cloud_source[src_idx].z;
    cor_tgt.col(i) << cloud_target[tgt_idx].x, cloud_target[tgt_idx].y, cloud_target[tgt_idx].z;
  }

  cor_src_pts=cor_src; // make a copy to use for visualization
  cor_tgt_pts=cor_tgt;

  std::cout <<"Copied Points Debug"<< std::endl;
  std::cout <<"Index, Index Query, Index Match"<< std::endl;

  for (int i = 0; i < P; ++i)
    std::cout <<i<<","<<cor_src_pts.col(i)<< std::endl;  

  Eigen::MatrixXd soln_T(4,4); // a Transformation matrix for the teaser solution 
 // soln_T<<soln.rotation(0,0),soln.rotation(0,1),soln.rotation(0,2),soln.translation(0),
 //         soln.rotation(1,0),soln.rotation(1,1),soln.rotation(1,2),soln.translation(1),
 //         soln.rotation(2,0),soln.rotation(2,1),soln.rotation(2,2),soln.translation(2),
 //         0                 ,0                 ,0                 ,1                  ;
<<<<<<< HEAD

  // identity is placeholder
  soln_T<<1,0,0,0,
          0,1,0,0,
          0,0,1,0,
          0,0,0,1 ;        

  Eigen::MatrixXd soln_T_inv(4,4);
  soln_T_inv=soln_T.inverse(); // take the inverse of the transformation returned by Teaser

  // This part seems very over bloated !!! 
  // I feel like this is done in a method somewhere - manually converting from TF to EIGEN

  tf::Quaternion q_result;
  tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);

  tf::Quaternion q_inverse;
  tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
  // instantiate a 3x3 rotation matrix from the transformation matrix // 
  

=======

  // identity is placeholder
  soln_T<<1,0,0,0,
          0,1,0,0,
          0,0,1,0,
          0,0,0,1 ;        

  Eigen::MatrixXd soln_T_inv(4,4);
  soln_T_inv=soln_T.inverse(); // take the inverse of the transformation returned by Teaser

  // This part seems very over bloated !!! 
  // I feel like this is done in a method somewhere - manually converting from TF to EIGEN

  tf::Quaternion q_result;
  tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);

  tf::Quaternion q_inverse;
  tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
  // instantiate a 3x3 rotation matrix from the transformation matrix // 
  

>>>>>>> master
  tf::Matrix3x3 R_result(soln_T(0,0),soln_T(0,1),soln_T(0,2),
                         soln_T(1,0),soln_T(1,1),soln_T(1,2),
                         soln_T(2,0),soln_T(2,1),soln_T(2,2));
  tf2::Matrix3x3 R_result_tf2(soln_T(0,0),soln_T(0,1),soln_T(0,2),
                              soln_T(1,0),soln_T(1,1),soln_T(1,2),
                              soln_T(2,0),soln_T(2,1),soln_T(2,2));
  
  tf::Matrix3x3 R_inverse(soln_T_inv(0,0),soln_T_inv(0,1),soln_T_inv(0,2),
                          soln_T_inv(1,0),soln_T_inv(1,1),soln_T_inv(1,2),
                          soln_T_inv(2,0),soln_T_inv(2,1),soln_T_inv(2,2));
  tf2::Matrix3x3 R_inverse_tf2( soln_T_inv(0,0),soln_T_inv(0,1),soln_T_inv(0,2),
                                soln_T_inv(1,0),soln_T_inv(1,1),soln_T_inv(1,2),
                                soln_T_inv(2,0),soln_T_inv(2,1),soln_T_inv(2,2));
  
  // copy tf::quaternion from R_result to q_result
  R_result.getRotation(q_result);
  R_result_tf2.getRotation(*q_result_tf2);
  q_result_tf2->normalize(); // normalize the Quaternion

  // copy tf::quaternion from R_inverse to q_inverse
  R_inverse.getRotation(q_inverse);
  R_inverse_tf2.getRotation(*q_inverse_tf2);
  q_inverse_tf2->normalize(); // normalize the Quaternion

  // set rotation and origin of a quaternion for the tf transform object
  T_AB.setRotation(q_result);
  T_AB.setOrigin(tf::Vector3(soln_T(0),soln_T(1),soln_T(2)));
 
  // set rotation and origin of a quaternion for the tf transform object
  T_BA.setRotation(q_inverse);
  T_BA.setOrigin(tf::Vector3(soln_T_inv(0,3),soln_T_inv(1,3),soln_T_inv(2,3)));
  
  tf::transformStampedTFToMsg(T_AB,msg_AB);
  tf::transformStampedTFToMsg(T_BA,msg_BA);

  std::cout << "END OF REGISTER_CLOUD_PCL FUNCTION" << std::endl;
}
*/


/*
  //instantiate pub for the points and line marker
  ros::Publisher pub_pointslines = node.advertise<visualization_msgs::Marker>("marker_pointslines",10);

  visualization_msgs::Marker points_src, points_tgt, line_strip, line_list;
  points_src.header.frame_id = points_tgt.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "base_link";
  points_src.header.stamp = points_tgt.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points_src.ns = points_tgt.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points_src.action = points_tgt.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points_src.pose.orientation.w = points_tgt.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points_src.id = 0;
  points_tgt.id = 1;
  line_strip.id = 2;
  line_list.id = 3;

  points_src.type = visualization_msgs::Marker::POINTS;
  points_tgt.type = visualization_msgs::Marker::POINTS;
  
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points_src.scale.x = 0.002;
  points_src.scale.y = 0.002;
  //points_src.scale.z = 0.002;
  points_tgt.scale.x = 0.002;
  points_tgt.scale.y = 0.002;
  //points_tgt.scale.z = 0.002;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.001;
  line_list.scale.x = 0.001;

  // Points are green
  points_src.color.g = 1.0f;
  points_src.color.a = 1.0;
  points_tgt.color.r = 1.0f;
  points_tgt.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  float f = 0.0;
  // Create the vertices for the points and lines
  for (uint32_t i = 0; i < 100; ++i)
  {
    //float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    //float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

    cor_src_pts(1,i);

    geometry_msgs::Point p_src,p_tgt; 
    p_src.x = cor_src_pts(0,i); // first point of each line is on source cloud
    p_src.y = cor_src_pts(1,i);
    p_src.z = cor_src_pts(2,i);

    p_tgt.x = cor_tgt_pts(0,i); // first point of each line is on source cloud
    p_tgt.y = cor_tgt_pts(1,i);
    p_tgt.z = cor_tgt_pts(2,i);

    points_src.points.push_back(p_src);
    points_tgt.points.push_back(p_tgt);
    //line_strip.points.push_back(p_src);

    // The line list needs two points for each line
    //line_list.points.push_back(p);
    //p.x += cor_tgt_pts(0,i); // the second point of each line is on target cloud
    //p.y += cor_tgt_pts(1,i);
    //p.z += cor_tgt_pts(2,i);

    //line_list.points.push_back(p);
  }
  */