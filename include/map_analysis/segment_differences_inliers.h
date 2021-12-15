#pragma once
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/search/search.h> // for Search
#include <pcl/segmentation/segment_differences.h>
#include <vector>
#include <string>

namespace pcl {
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain the difference between two aligned point clouds as another point cloud, given a distance threshold.
  * \param src the input point cloud source
  * \param threshold the distance threshold (tolerance) for point correspondences. (e.g., check if f a point p1 from 
  * src has a correspondence > threshold than a point p2 from tgt)
  * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching built over the target cloud
  * \param output the resultant output point cloud difference
  * \ingroup segmentation
  */
template <typename PointT>
void PCDiffInliers_subdomain(const pcl::PointCloud<PointT>& src,
                             const pcl::PointCloud<PointT> &tgt,
                             int istart, int ipoints,
                             double threshold,
                             const typename boost::shared_ptr<pcl::search::Search<PointT> > &tree,
                             std::vector<int> &src_indices,
                             std::vector<int> &inlier_indices);
template <typename PointT>
  void getPointCloudDifferenceInliers(
      const pcl::PointCloud<PointT> &src, const pcl::PointCloud<PointT> &tgt,
      double threshold, const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
      pcl::PointCloud<PointT> &output, pcl::PointCloud<PointT>& inliers);

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SegmentDifferences obtains the difference between two spatially
  * aligned point clouds and returns the difference between them for a maximum
  * given distance threshold.
  * \author Radu Bogdan Rusu
  * \ingroup segmentation
  */
template <typename PointT>
class SegmentDifferencesInliers: public PCLBase<PointT> {
  typedef PCLBase<PointT> BasePCLBase;

 public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef typename pcl::search::Search<PointT> KdTree;
  typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

  typedef PointIndices::Ptr PointIndicesPtr;
  typedef PointIndices::ConstPtr PointIndicesConstPtr;

  /** \brief Empty constructor. */
  SegmentDifferencesInliers() :
    tree_(), target_(), distance_threshold_(0)
  {};

  /** \brief Provide a pointer to the target dataset against which we
   * compare the input cloud given in setInputCloud
   *
   * \param cloud the target PointCloud dataset
   */
  inline void
    setTargetCloud(const PointCloudConstPtr &cloud) { target_ = cloud; }

  /** \brief Get a pointer to the input target point cloud dataset. */
  inline PointCloudConstPtr const
    getTargetCloud() { return (target_); }

  /** \brief Provide a pointer to the search object.
   * \param tree a pointer to the spatial search object.
   */
  inline void
    setSearchMethod(const KdTreePtr &tree) { tree_ = tree; }

  /** \brief Get a pointer to the search method used. */
  inline KdTreePtr
    getSearchMethod() { return (tree_); }

  /** \brief Set the maximum distance tolerance (squared) between corresponding
   * points in the two input datasets.
   *
   * \param sqr_threshold the squared distance tolerance as a measure in L2 Euclidean space
   */
  inline void
    setDistanceThreshold(double sqr_threshold) { distance_threshold_ = sqr_threshold; }

  /** \brief Get the squared distance tolerance between corresponding points as a
   * measure in the L2 Euclidean space.
   */
  inline double
    getDistanceThreshold() { return (distance_threshold_); }

  /** \brief Segment differences between two input point clouds.
   * \param output the resultant difference between the two point clouds as a PointCloud
   */
  void
    segment(PointCloud &output);

  void
    segmentInliers(PointCloud &output, PointCloud& inliers);

 protected:
  // Members derived from the base class
  using BasePCLBase::input_;
  using BasePCLBase::indices_;
  using BasePCLBase::initCompute;
  using BasePCLBase::deinitCompute;

  /** \brief A pointer to the spatial search object. */
  KdTreePtr tree_;

  /** \brief The input target point cloud dataset. */
  PointCloudConstPtr target_;

  /** \brief The distance tolerance (squared) as a measure in the L2
   * Euclidean space between corresponding points. 
   */
  double distance_threshold_;

  /** \brief Class getName method. */
  virtual std::string
    getClassName() const { return ("SegmentDifferencesInliers"); }
};
} // namespace pcl

#include <map_analysis/segment_differences_inliers.hpp>
