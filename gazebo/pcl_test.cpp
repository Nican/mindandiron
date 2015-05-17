#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pointcloud.h"
#include "msgpack.h"

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

static void UpdatePointCloud2(const Robot::DepthImgData &imgData, PointCloudT &cloud)
{
  //cloud.points.resize(0);
  cloud.is_dense = true;
  //cloud.width = imgData.width;
  //cloud.height = imgData.height;

  //float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = imgData.hfov;
  double fl = ((double)imgData.width) / (2.0 *tan(hfov/2.0));

  //double fl2 = ((double)imgData.height) / (2.0 *tan(60.0/2.0));

  // convert depth to point cloud
  //cloud.points.resize(imgData.data.size());
  cloud.points.resize(0);
  for (uint32_t j=0; j<imgData.height; j++)
  {
    double pAngle;

    //if (imgData.height>1) 
      pAngle = ((double)j - 0.5*(double)(imgData.height-1)) / fl; //atan2( (double)j - 0.5*(double)(imgData.height-1), fl);
    //else            
    //  pAngle = 0.0;

      for (uint32_t i=0; i<imgData.width; i++)
      {
        double yAngle;
      //if (imgData.width>1) 
        yAngle = ((double)i - 0.5*(double)(imgData.width-1)) / fl; //atan2( (double)i - 0.5*(double)(imgData.width-1), fl);
      //else            
      //  yAngle = 0.0;

      double depth = imgData.data[index++]; // + 0.0*this->myParent->GetNearClip();

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      PointNT point;
      point.x      = -depth * yAngle; //tan(yAngle);
      point.y      = -depth * pAngle; //tan(pAngle);
      //if(depth > this->point_cloud_cutoff_)

      if(depth > 0.01)
      {
        point.z    = depth;
        cloud.push_back(point);
      }
      else //point in the unseeable range
      {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        cloud.is_dense = false;
      }

      //point.r = 255;
      //point.g = (char) (std::min(1.0, std::max((point.z + 3.0) / 6.0, 0.0)) * 255.0);

      //cloud.points[i + j * imgData.width] = point;

    }
  }

}

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  Robot::DepthImgData imgData;
  imgData.width = 512;
  imgData.height = 424;
  imgData.hfov = 70.6 / 180.0 * M_PI;
  imgData.data.resize(imgData.width * imgData.height);
  FILE* pFile;

  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    //return (1);
  }

  char basePath[] = "/home/nican/zfs/storage/kinect/kinect2";
  char path[5000];
  int id = 240;


  sprintf(path, "%s/depth_%d.bin", basePath, id);
  pFile = fopen (path , "rb");
  fread (&imgData.data.at(0), 1, imgData.data.size() * sizeof(float),pFile);
  fclose (pFile);

  for(auto &point : imgData.data)
      point /= 1000.0;
    
  UpdatePointCloud2(imgData, *object);

  sprintf(path, "%s/depth_%d.bin", basePath, id+1);
  pFile = fopen (path , "rb");
  fread (&imgData.data.at(0), 1, imgData.data.size() * sizeof(float),pFile);
  fclose (pFile);

  for(auto &point : imgData.data)
      point /= 1000.0;
    
  UpdatePointCloud2(imgData, *scene);


  /*
  sprintf(path, "%s/segmented_%d.pcd", basePath, id);
  pcl::io::loadPCDFile(path, *object);

  sprintf(path, "%s/segmented_%d.pcd", basePath, id+1);
  pcl::io::loadPCDFile(path, *scene);
  */
  
  // Load object and scene
  /*
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
      pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }
  */
  
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.02f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.04);
  nest.setInputCloud (scene);
  nest.compute (*scene);

  nest.setInputCloud (object);
  nest.compute (*object);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.1);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
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
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }
  
  return (0);
}