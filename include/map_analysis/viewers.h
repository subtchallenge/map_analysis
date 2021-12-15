#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> ptcld_merge_vis (
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
    const std::vector<typename pcl::PointCloud<Point_T>::Ptr>& clouds, 
    const std::vector<pcl::PointCloud<pcl::Normal>::Ptr>& normals,
    const std::vector<typename pcl::PointCloud<Point_T>::Ptr>& keypoints, 
    const typename pcl::PointCloud<Point_T>::Ptr& fullcloud);

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
twoport_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer ,
	    const typename pcl::PointCloud<Point_T>::Ptr& cloud1, 
	    int cloudnum,
	    const std::vector<typename std::pair<std::pair<double,int>, typename pcl::PointCloud<Point_T>::Ptr> >& matches);

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
single_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud);

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
badass_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                const std::string& text, 
		const typename pcl::PointCloud<Point_T>::Ptr& cloud);

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
badass_mega_vis_velodyne(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                         const std::string& text, 
                         const typename pcl::PointCloud<Point_T>::Ptr& cloud);

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
double_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud1,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud2);
