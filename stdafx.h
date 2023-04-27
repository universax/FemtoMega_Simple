#pragma once
// std
#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <functional>

// CV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

//Boost
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/property_tree/xml_parser.hpp>

//PCL
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/tracking/tracker.h>
#include <pcl/tracking/pyramidal_klt.h>

#include <pcl/range_image/range_image.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>

#include <pcl/keypoints/harris_2d.h>


// Typedef
#include "Typedef.h"