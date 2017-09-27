#include "laser_grasp/cloud_registration.h"

CloudRegistration::CloudRegistration(std::string main_scene_path, std::vector<std::string> scene_paths, std::vector<std::string> scene_paths2 ){
  std::cout<<"Entro funcion"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(main_scene_path, *main_cloud)==-1){
    std::cout<<"Couldn't read file "<<main_scene_path<<std::endl;
  }
  else{
    main_scene_=filterCloud(main_cloud);
    //showCloud(main_scene_);
  }
  for(int i=0; i< scene_paths.size(); i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(scene_paths[i], *cloud)==-1){
      std::cout<<"Couldn't read file "<<scene_paths[i]<<std::endl;
    }
    else{
      scenes_.push_back(filterCloud(cloud));
      //showCloud(cloud);
    }
  }
  for(int i=0; i< scene_paths2.size(); i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(scene_paths2[i], *cloud)==-1){
      std::cout<<"Couldn't read file "<<scene_paths2[i]<<std::endl;
    }
    else{
      scenes2_.push_back(filterCloud(cloud));
      //showCloud(cloud);
    }
  }
}


CloudRegistration::CloudRegistration(){

}

void CloudRegistration::showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.addPointCloud<pcl::PointXYZ>(cloud);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();

}

void CloudRegistration::showCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.addPointCloud<pcl::PointNormal>(cloud);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();

}

void CloudRegistration::showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud1");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud2, single_color2, "cloud2");
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudRegistration::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting filtering...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.5, 2.0);
  pass.filter(*cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  //for(int i=0; i<=1000; i+=50){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered2);
    //showCloud(cloud_filtered2);
  //}
  return cloud_filtered2;

}



bool distance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2, float distance){

  return true;
}

std::vector<pcl::PointIndices> CloudRegistration::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting segmentation...\n");

  //showCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);


  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  //pcl::ConditionalEuclideanClustering<pcl::PointXYZ> ec;
  //pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
  ec.setClusterTolerance (0.03); // 1cm

  ec.setMinClusterSize (100);

  ec.setMaxClusterSize (25000);
  //ec.setConditionFunction(&distance);

  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);


  ec.extract (cluster_indices);
  //ec.segment(*clusters);

  //std::cout<<clusters->size()<<std::endl;


  //cluster_indices=*clusters;
  for(int i=0; i<cluster_indices.size(); i++){
    std::cout<<"Cluster "<<cluster_indices[i].indices.size()<<std::endl;
  }



  return cluster_indices;
}

std::vector<pcl::PointIndices> CloudRegistration::segmentCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting segmentation...\n");


   pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
   pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
   normal_estimator.setSearchMethod (tree);
   normal_estimator.setInputCloud (cloud);
   normal_estimator.setKSearch (50);
   normal_estimator.compute (*normals);

   /*pcl::IndicesPtr indices (new std::vector <int>);
   pcl::PassThrough<pcl::PointXYZ> pass;
   pass.setInputCloud (cloud);
   pass.setFilterFieldName ("z");
   pass.setFilterLimits (0.0, 1.0);
   pass.filter (*indices);*/

   pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
   reg.setMinClusterSize (100);
   reg.setMaxClusterSize (1000);
   reg.setSearchMethod (tree);
   reg.setNumberOfNeighbours (50);
   reg.setInputCloud (cloud);
   //reg.setIndices (indices);
   reg.setInputNormals (normals);
   reg.setSmoothnessThreshold (9.0 / 180.0 * M_PI);
   reg.setCurvatureThreshold (3.0);

   std::vector <pcl::PointIndices> clusters;
   reg.extract (clusters);

   std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
   std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
   std::cout << "These are the indices of the points of the initial" <<
     std::endl << "cloud that belong to the first cluster:" << std::endl;
   int counter = 0;
   while (counter < clusters[0].indices.size ())
   {
     std::cout << clusters[0].indices[counter] << ", ";
     counter++;
     if (counter % 10 == 0)
       std::cout << std::endl;
   }
   std::cout << std::endl;

   pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
   pcl::visualization::CloudViewer viewer ("Cluster viewer");
   //viewer.showCloud(colored_cloud);
   while (!viewer.wasStopped ())
   {
   }

   return clusters;
}

std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> CloudRegistration::getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Getting the plane...\n");
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> pair(inliers, coefficients);

    return pair;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudRegistration::getObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n_objects){
  pcl::console::print_highlight ("Getting objects...\n");
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects;
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> pair=getPlane(cloud);


  pcl::PointIndices::Ptr plane_inliers=pair.first;
  pcl::ModelCoefficients::Ptr coefficients=pair.second;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());


  if(plane_inliers->indices.size()==0){
    pcl::copyPointCloud(*cloud, *cloud_no_plane);
  }
  else{
    pcl::ExtractIndices<pcl::PointXYZ> extract(true);
    extract.setInputCloud(cloud);
    extract.setIndices(plane_inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_plane);

    pcl::ExtractIndices<pcl::PointXYZ> extract2(true);
    extract2.setInputCloud(cloud);
    extract2.setIndices(plane_inliers);
    extract2.setNegative(false);
    extract2.filter(*cloud_plane);

  }


  std::vector<pcl::PointIndices> object_indices=segmentCloud(cloud_no_plane);


  for(int i=0; i<n_objects; i++){
  //for(int i=0; i<object_indices.size(); i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::copyPointCloud(*cloud_no_plane, *cloud_aux);
     cloud_aux->points.clear();
     for(int j=object_indices[i].indices.size()-1; j>=0; j--){
       pcl::PointXYZ point(cloud_aux->points[object_indices[i].indices[j]].x, cloud_aux->points[object_indices[i].indices[j]].y, cloud_aux->points[object_indices[i].indices[j]].z);
       cloud_aux->points.push_back(point);
     }
     cloud_aux->width=cloud_aux->points.size();
     std::cout<<object_indices[i].indices.size()<<" "<<cloud_aux->width<<std::endl;
     //showCloud(cloud_aux);
     //showCloud(cloud_no_plane);

    /* pcl::ProjectInliers<pcl::PointXYZ> proj;
     proj.setModelType (pcl::SACMODEL_PLANE);
     proj.setInputCloud (cloud_aux);
     proj.setModelCoefficients (coefficients);
     proj.filter (*cloud_aux);*/
     //showClouds(cloud_no_plane, cloud_aux);

     *cloud_aux+=*cloud_plane;
     objects.push_back(cloud_aux);
  }
  return objects;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudRegistration::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting downsampling...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud_downsampled);

  return cloud_downsampled;
}
pcl::PointCloud<pcl::PointNormal>::Ptr CloudRegistration::downsampleCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){
  pcl::console::print_highlight ("Starting downsampling...\n");
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointNormal>);
  pcl::VoxelGrid<pcl::PointNormal> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud_downsampled);

  return cloud_downsampled;
}

pcl::PointCloud<pcl::PointNormal>::Ptr CloudRegistration::alignCloud(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object){
  pcl::console::print_highlight ("Aligning clouds...\n");

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointCloud<pcl::PointNormal>::Ptr new_scene(new  pcl::PointCloud<pcl::PointNormal>);

  const float leaf = 0.005f;

  pcl::console::print_highlight ("Estimating features...\n");
  pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.1);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
    std::cout<<"Good alignment? y/n"<<std::endl;
    std::string answer;
    std::cin>>answer;
    if(answer=="y"){
    //if(true){
      pcl::copyPointCloud(*scene, *new_scene);
      *new_scene+=*object_aligned;
      return new_scene;
    }
    else{
      return scene;
    }
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return scene;
  }
}


Eigen::Matrix4f CloudRegistration::alignCloud_transformation(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object){
  pcl::console::print_highlight ("Aligning clouds...\n");

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointCloud<pcl::PointNormal>::Ptr new_scene(new  pcl::PointCloud<pcl::PointNormal>);

  const float leaf = 0.005f;

  pcl::console::print_highlight ("Estimating features...\n");
  pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.1);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  Eigen::Matrix4f transformation_failure;
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // Show alignment
    /*pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();*/
   // std::cout<<"Good alignment? y/n"<<std::endl;

    //std::string answer;
    //std::cin>>answer;
    //if(answer=="y"){


      return transformation;

  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return transformation_failure;
  }
}

pcl::PointCloud<pcl::PointNormal>::Ptr CloudRegistration::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Getting normals...\n");


  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  ne.setRadiusSearch (0.01);

  ne.compute (*normals);
  pcl::concatenateFields(*cloud, *normals, *cloud_normals);

  return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudRegistration::addPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool main){
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> pair=getPlane(cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_plane);
  if(main){
    for(float x=-1; x<=1; x+=0.01){
      for(float y=-1; y<=1; y+=0.01){
        float z=-((pair.second->values[0]*x+pair.second->values[1]*y+pair.second->values[3])/pair.second->values[2]);
        pcl::PointXYZ point(x, y, z);
        cloud_plane->points.push_back(point);
        cloud_plane->width++;

      }
    }
  }
  else{
    for(float x=-0.5; x<=0.5; x+=0.01){
      for(float y=-0.5; y<=0.5; y+=0.01){
        float z=-((pair.second->values[0]*x+pair.second->values[1]*y+pair.second->values[3])/pair.second->values[2]);
        pcl::PointXYZ point(x, y, z);
        cloud_plane->points.push_back(point);
        cloud_plane->width++;

      }
    }
  }
  showCloud(downsampleCloud(cloud_plane));
  return cloud_plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudRegistration::registerClouds(int n_objects){
  std::cout<<"Main scene"<<std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr main_scene_downsampled=downsampleCloud(main_scene_);
  pcl::PointCloud<pcl::PointNormal>::Ptr main_scene_normals=getNormals(main_scene_downsampled);
  for(int i=0; i<scenes_.size(); i++){
    std::cout<<"Scene "<<i<<std::endl;
     pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled=downsampleCloud(scenes_[i]);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled=scenes_[i];
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects_scene=getObjects(scene_downsampled, n_objects);
    for(int j=0; j<objects_scene.size(); j++){
      std::cout<<"Object "<<j<<std::endl;
      //pcl::PointCloud<pcl::PointXYZ>::Ptr object_downsampled=downsampleCloud(objects_scene[j]);

      pcl::PointCloud<pcl::PointNormal>::Ptr object_normals=getNormals(scene_downsampled);

      main_scene_normals=alignCloud(main_scene_normals, object_normals);
      main_scene_normals=downsampleCloud(main_scene_normals);

    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr completed_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*main_scene_normals, *completed_scene);
  showCloud(completed_scene);
  return completed_scene;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudRegistration::registerClouds_onebyone(int n_objects){
  std::cout<<"Main scene"<<std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr main_scene_downsampled=downsampleCloud(main_scene_);
  pcl::PointCloud<pcl::PointNormal>::Ptr main_scene_normals=getNormals(main_scene_downsampled);

  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> scenes_normals, scenes2_normals;

  for(int i=0; i<scenes_.size(); i++){
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_normals=getNormals(downsampleCloud(scenes_[i]));
    scenes_normals.push_back(scene_normals);
  }
  for(int i=0; i<scenes2_.size(); i++){
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_normals=getNormals(downsampleCloud(scenes2_[i]));
    scenes2_normals.push_back(scene_normals);
  }

  std::vector<Eigen::Matrix4f> trans1, trans2;

  for(int i=0; i<scenes_.size()-1; i++){
    Eigen::Matrix4f trans=alignCloud_transformation(scenes_normals[i], scenes_normals[i+1]);
    trans1.push_back(trans);
  }
  for(int i=0; i<scenes2_.size()-1; i++){
    Eigen::Matrix4f trans=alignCloud_transformation(scenes2_normals[i], scenes2_normals[i+1]);
    trans2.push_back(trans);
  }

  Eigen::Matrix4f trans_main1=alignCloud_transformation(main_scene_normals, scenes_normals[0]);
  Eigen::Matrix4f trans_main2=alignCloud_transformation(main_scene_normals, scenes2_normals[0]);


  /*pcl::PointCloud<pcl::PointNormal>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointNormal>);

  pcl::transformPointCloudWithNormals(*scenes_normals[0], *aux_cloud, trans_main1);
  *main_scene_normals+=*aux_cloud;
  showCloud(main_scene_normals);

  for(int i=0; i<trans1.size(); i++){
    trans_main1=trans1[i]*trans_main1;
    pcl::transformPointCloudWithNormals(*scenes_normals[i+1], *aux_cloud, trans_main1);
    *main_scene_normals+=*aux_cloud;
    showCloud(main_scene_normals);
  }

  pcl::transformPointCloudWithNormals(*scenes2_normals[0], *aux_cloud, trans_main2);
  *main_scene_normals+=*aux_cloud;
  showCloud(main_scene_normals);

  for(int i=0; i<trans2.size(); i++){
    trans_main2=trans2[i]*trans_main2;
    pcl::transformPointCloudWithNormals(*scenes2_normals[i+1], *aux_cloud, trans_main2);
    *main_scene_normals+=*aux_cloud;
    showCloud(main_scene_normals);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr completed_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*main_scene_normals, *completed_scene);
  showCloud(completed_scene);*/


  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(*scenes_[0], *aux_cloud, trans_main1);
  *main_scene_+=*aux_cloud;
  showCloud(main_scene_);

  for(int i=0; i<trans1.size(); i++){
    trans_main1=trans1[i]*trans_main1;
    pcl::transformPointCloud(*scenes_[i+1], *aux_cloud, trans_main1);
    *main_scene_+=*aux_cloud;
    showCloud(main_scene_);
  }

  pcl::transformPointCloud(*scenes2_[0], *aux_cloud, trans_main2);
  *main_scene_+=*aux_cloud;
  showCloud(main_scene_);

  for(int i=0; i<trans2.size(); i++){
    trans_main2=trans2[i]*trans_main2;
    pcl::transformPointCloud(*scenes2_[i+1], *aux_cloud, trans_main2);
    *main_scene_+=*aux_cloud;
    showCloud(main_scene_);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr completed_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*main_scene_, *completed_scene);
  completed_scene=filterCloud(completed_scene);
  showCloud(completed_scene);
  return completed_scene;
}
