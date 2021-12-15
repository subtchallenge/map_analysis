#pragma once
#include <map_analysis/segment_differences_inliers.h>

#include <pcl/common/io.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree
#include <vector>
#include <omp.h>

template <typename PointT> void
pcl::PCDiffInliers_subdomain(const pcl::PointCloud<PointT>& src,
                                  const pcl::PointCloud<PointT> &tgt, 
                                  int istart, int ipoints,
                                  double threshold,
                                  const typename boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                                  std::vector<int> &src_indices,
                                  std::vector<int> &inlier_indices) {
  int src_ind = 0;
  int inlier_ind = 0;

  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);

  for (int i = istart; i < istart + ipoints; ++i) {
    if (!isFinite (src.points[i]))
      continue;
    // Search for the closest point in the target data set (number of neighbors to find = 1)
    if (!tree->nearestKSearch(src.points[i], 1, nn_indices, nn_distances)) {
      continue;
    }

    if (nn_distances[0] > threshold) {
      src_indices[src_ind++] = i;
    } else {
      inlier_indices[inlier_ind++] = i;
    }
  }
  src_indices.resize(src_ind);
  inlier_indices.resize(inlier_ind);
}
template <typename PointT> void
pcl::getPointCloudDifferenceInliers(const pcl::PointCloud<PointT> &src,
                                    const pcl::PointCloud<PointT> &tgt, 
                                    double threshold,
                                    const typename boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                                    pcl::PointCloud<PointT> &output,
                                    pcl::PointCloud<PointT> &inliers) {
  //ROS_ERROR_STREAM("Using multithreaded change detection");
  std::vector<int> src_indices(src.points.size());
  int src_ind = 0;
  std::vector<int> inlier_indices(src.points.size());
  int inlier_ind = 0;
  int iam, nt, ipoints, istart; 
  omp_set_num_threads(9);
#pragma omp parallel default(shared) private(iam,nt,ipoints,istart) 
  {
    iam = omp_get_thread_num(); 
    nt = omp_get_num_threads();
    ipoints = src.points.size() / nt; /* size of partition */
    //std::cout <<"Thread " << iam << " processing " << ipoints << " points" << std::endl;
    istart = iam * ipoints; /* starting array index */
    if (iam == nt-1) /* last thread may do more */
      ipoints = src.points.size() - istart;
    std::vector<int> src_indices_i(ipoints);
    std::vector<int> inlier_indices_i(ipoints);
    PCDiffInliers_subdomain<PointT>(src, tgt, istart, ipoints, threshold, tree, src_indices_i, inlier_indices_i);
#pragma omp critical (results)
    {
      for (int i = 0; i < src_indices_i.size(); i++) {
        src_indices[src_ind++] = src_indices_i[i];
      }
      for (int i = 0; i < inlier_indices_i.size(); i++) {
        inlier_indices[inlier_ind++] = inlier_indices_i[i];
      }
    }
  }
  src_indices.resize(src_ind);
  inlier_indices.resize(inlier_ind);
  // Allocate enough space and copy the basics
  output.points.resize(src_indices.size());
  output.header   = src.header;
  output.width    = static_cast<std::uint32_t>(src_indices.size());
  output.height   = 1;
  output.is_dense = true;
  copyPointCloud(src, src_indices, output);
  copyPointCloud(src, inlier_indices, inliers);
}

/*template <typename PointT> void
pcl::getPointCloudDifferenceInliers(const pcl::PointCloud<PointT> &src,
                                    const pcl::PointCloud<PointT> &tgt, 
                                    double threshold,
                                    const typename boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                                    pcl::PointCloud<PointT> &output,
                                    pcl::PointCloud<PointT> &inliers) {
  // We're interested in a single nearest neighbor only
  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);

  // The src indices that do not have a neighbor in tgt
  std::vector<int> src_indices(src.points.size());
  int src_ind = 0;
  std::vector<int> inlier_indices(src.points.size());
  int inlier_ind = 0;

  // Iterate through the source data set
  for (int i = 0; i < static_cast<int> (src.points.size()); ++i) {
    if (!isFinite (src.points[i]))
      continue;
    // Search for the closest point in the target data set (number of neighbors to find = 1)
    if (!tree->nearestKSearch(src.points[i], 1, nn_indices, nn_distances)) {
//      PCL_WARN("No neighbor found for point %lu (%f %f %f)!\n", i, src.points[i].x, src.points[i].y, src.points[i].z);
      continue;
    }

    if (nn_distances[0] > threshold) {
      src_indices[src_ind++] = i;
    } else {
      inlier_indices[inlier_ind++] = i;
    }
  }
  src_indices.resize(src_ind);
  inlier_indices.resize(inlier_ind);
  // Allocate enough space and copy the basics
  output.points.resize(src_indices.size());
  output.header   = src.header;
  output.width    = static_cast<uint32_t>(src_indices.size());
  output.height   = 1;
  output.is_dense = true;
  copyPointCloud(src, src_indices, output);
  copyPointCloud(src, inlier_indices, inliers);
}
*/
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SegmentDifferencesInliers<PointT>::segment(PointCloud &output) {
  output.header = input_->header;

  if (!initCompute())  {
    output.width = output.height = 0;
    output.clear();
    return;
  }

  // If target is empty, input - target = input
  if (target_->points.empty()) {
    output = *input_;
    return;
  }

  // Initialize the spatial locator
  if (!tree_) {
    if (target_->isOrganized())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }
  // Send the input dataset to the spatial locator
  tree_->setInputCloud(target_);

  getPointCloudDifference (*input_, *target_, distance_threshold_, tree_, output);

  deinitCompute();
}//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SegmentDifferencesInliers<PointT>::segmentInliers(PointCloud &output,
                                                       PointCloud& inliers) {
  output.header = input_->header;

  if (!initCompute())  {
    output.width = output.height = 0;
    output.clear();
    return;
  }

  // If target is empty, input - target = input
  if (target_->points.empty()) {
    output = *input_;
    return;
  }

  // Initialize the spatial locator
  if (!tree_) {
    if (target_->isOrganized())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }
  // Send the input dataset to the spatial locator
  tree_->setInputCloud(target_);

  getPointCloudDifferenceInliers(*input_, *target_, distance_threshold_, tree_, output, inliers);
  deinitCompute();
}
