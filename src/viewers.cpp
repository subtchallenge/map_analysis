#include "viewers.h"
#define BG_R 1.0
#define BG_G 1.0
#define BG_B 1.0

template <class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
ptcld_merge_vis (
                 boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                 const std::vector<typename pcl::PointCloud<Point_T>::Ptr>& clouds, 
                 const std::vector<pcl::PointCloud<pcl::Normal>::Ptr>& normals,
                 const std::vector<typename pcl::PointCloud<Point_T>::Ptr>& keypoints, 
                 const typename pcl::PointCloud<Point_T>::Ptr& fullcloud) {
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  if (clouds.size() == 0) return viewer;
  //This function assumes that clouds will have a constant size
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Merge viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor(BG_R, BG_G, BG_B);
    float width = 1.0 / (float)clouds.size();
    for (unsigned int i = 0;i<clouds.size();i++) {
      int v1(i);
      viewer->createViewPort(i*width, 0.5, (i+1)*width, 1.0, v1);
      viewer->setBackgroundColor (BG_R, BG_G, BG_B);
      std::stringstream textstream;
      textstream << "Cloud #"<<i+1;
      std::stringstream ctrlstream;
      ctrlstream << "v"<<i+1<<" text";
      viewer->addText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str(), v1);
      pcl::visualization::PointCloudColorHandlerCustom<Point_T> 
	red(keypoints[i], 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
	height(clouds[i], std::string("z"));
      std::stringstream cloudstr;
      cloudstr <<"cloud"<<i+1;
      viewer->addPointCloud<Point_T> (clouds[i], height, cloudstr.str().c_str(), v1);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudstr.str().c_str());
      
      if (normals[i]) {
	std::stringstream normalstr;
	normalstr <<"normals"<<i+1;
	viewer->addPointCloudNormals<Point_T, pcl::Normal> (clouds[i], normals[i], 10, 0.5, normalstr.str().c_str(),v1);
      }
      if (keypoints[i]){
	std::stringstream kptstr;
	kptstr << "kpts"<<i+1;
	viewer->addPointCloud<Point_T> (keypoints[i], red, kptstr.str().c_str(), v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, kptstr.str().c_str());
	
      }
    }
    int v3(0);
    viewer->createViewPort(0.0, 0.0, 1.0, 0.5, v3);
    viewer->setBackgroundColor (BG_R, BG_G, BG_B);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v3);
    viewer->addText("Merged", 10, 10, "merged text", v3);
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(fullcloud, std::string("z"));
    
    viewer->addPointCloud<Point_T> (fullcloud, height, "fullcloud", v3);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fullcloud");
    viewer->addCoordinateSystem (1.0);
  }else {
    
    for (unsigned int i = 0;i<clouds.size();i++) {
      int v1(i);
      std::stringstream textstream;
      textstream << "Cloud #"<<i+1;
      std::stringstream ctrlstream;
      ctrlstream << "v"<<i+1<<" text";
      viewer->updateText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str());
      pcl::visualization::PointCloudColorHandlerCustom<Point_T> 
	red(keypoints[i], 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
	height(clouds[i], std::string("z"));
      std::stringstream cloudstr;
      cloudstr <<"cloud"<<i+1;
      viewer->updatePointCloud<Point_T> (clouds[i], height, cloudstr.str().c_str());
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudstr.str().c_str());
      
      if (normals[i]) {
	std::stringstream normalstr;
	normalstr <<"normals"<<i+1;
	//	viewer->updatePointCloudNormals<Point_T, pcl::Normal> (clouds[i], normals[i], 10, 0.5, normalstr.str().c_str());
      }
      if (keypoints[i]){
	std::stringstream kptstr;
	kptstr << "kpts"<<i+1;
	viewer->updatePointCloud<Point_T> (keypoints[i], red, kptstr.str().c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, kptstr.str().c_str());
      }
    }
    //    viewer->setBackgroundColor(BG_R, BG_G, BG_B);
    viewer->updateText("Merged", 10, 10, "merged text");
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(fullcloud, std::string("z"));
    
    viewer->updatePointCloud<Point_T> (fullcloud, height, "fullcloud");

  }
  return (viewer);
}

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
twoport_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer ,
	    const typename pcl::PointCloud<Point_T>::Ptr& cloud1, 
	    int cloudnum,
	    const std::vector<typename std::pair<std::pair<double,int>, typename pcl::PointCloud<Point_T>::Ptr> >& matches) {
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  if (matches.size() == 0) return viewer;
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(BG_R, BG_G, BG_B);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v1);
    std::stringstream textstream;
    textstream << "Cloud #"<<cloudnum;
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->addText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str(), v1);
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(cloud1, std::string("z"));
    std::stringstream cloudstr;
    cloudstr <<"cloud"<<0;
    viewer->addPointCloud<Point_T> (cloud1, height, cloudstr.str().c_str(), v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
    for (unsigned int i = 0; i < 3;i++) {
      int v2(0);
      double height = 1.0 / 3.0;
      viewer->createViewPort(0.5, i * height, 1.0, (i+1)*height, v2);
      viewer->setBackgroundColor (BG_R, BG_G, BG_B);
      //      viewer->setBackgroundColor (BG_R, BG_G, BG_B, v2);
      std::stringstream cloudstr;
      cloudstr <<"cloud"<<i+1;

      std::stringstream textctrlstream;
      textctrlstream << "txt"<<i<<" text";

      int k = i;
      if (k > matches.size()-1) k = matches.size() - 1 ;
      pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
	height2(matches[k].second, std::string("z"));
      std::stringstream matchstring;
      matchstring << "match"<<i;
      std::stringstream matchvalstring;
      matchvalstring << "Cloud "<<matches[k].first.second<<" Likelihood: "<<matches[k].first.first;
      viewer->addText(matchvalstring.str().c_str(), 10, 10, matchstring.str().c_str(), v2);

      viewer->addPointCloud<Point_T> (matches[k].second, height2, cloudstr.str().c_str(), v2);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
    }
    viewer->addCoordinateSystem (1.0);
  }else {
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(cloud1, std::string("z"));
    viewer->updatePointCloud(cloud1, height, "cloud0");
    std::stringstream textstream;
    textstream << "Cloud #"<<cloudnum;
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->updateText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str());

    for (unsigned int i = 0;i<3;i++) {
      int k = i;
      if (k > matches.size() - 1) k = matches.size() - 1;
      pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
	height2(matches[k].second, std::string("z"));
      std::stringstream matchstring;
      matchstring << "match"<<i;
      std::stringstream matchvalstring;
      matchvalstring << "Cloud "<< matches[k].first.second <<" Likelihood: "<<matches[k].first.first;
      viewer->updateText(matchvalstring.str().c_str(), 10,10,matchstring.str().c_str());
      std::stringstream cloudstr;
      cloudstr <<"cloud"<<i+1;

      viewer->updatePointCloud(matches[k].second, height2, cloudstr.str().c_str());
    }
  }
  return (viewer);
}

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
single_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer ,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud) {
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->setBackgroundColor(BG_R, BG_G, BG_B);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor (BG_R, BG_G, BG_B);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v1);
    std::stringstream textstream;
    textstream << "Cloud #"<<0;
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->addText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str(), v1);
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(cloud, std::string("z"));
    std::stringstream cloudstr;
    cloudstr <<"cloud"<<0;
    viewer->addPointCloud<Point_T> (cloud, height, cloudstr.str().c_str(), v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
  }else {
    //Update
    std::stringstream cloudstr;
    cloudstr << "cloud"<<0;
    viewer->updatePointCloud(cloud, cloudstr.str().c_str());
  }
  return viewer;
}

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
badass_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                const std::string& text, 
		const typename pcl::PointCloud<Point_T>::Ptr& cloud) {
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->setBackgroundColor(0,0,0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor (0,0,0);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v1);
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->addText(text.c_str(), 10, 10, ctrlstream.str().c_str(), v1);
    //    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
    //      height(cloud, std::string("z"));
    std::stringstream cloudstr;
    cloudstr <<"cloud"<<0;
    viewer->addPointCloud<Point_T> (cloud, cloudstr.str().c_str(), v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
  }else {
    //Update
    std::stringstream cloudstr;
    cloudstr << "cloud"<<0;
    viewer->updatePointCloud(cloud, cloudstr.str().c_str());
  }
  return viewer;
}

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
badass_mega_vis_velodyne(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                const std::string& text, 
		const typename pcl::PointCloud<Point_T>::Ptr& cloud) {
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->setBackgroundColor(0,0,0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor (0,0,0);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v1);
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->addText(text.c_str(), 10, 10, ctrlstream.str().c_str(), v1);
    //    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
    //      height(cloud, std::string("z"));
    std::stringstream cloudstr;
    cloudstr <<"cloud"<<0;
    viewer->addPointCloud<Point_T> (cloud, cloudstr.str().c_str(), v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
  }else {
    //Update
    std::stringstream cloudstr;
    cloudstr << "cloud"<<0;
    viewer->updatePointCloud<Point_T>(cloud, cloudstr.str().c_str());
  }
  return viewer;
}

template<class Point_T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
double_mega_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer ,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud1,
		const typename pcl::PointCloud<Point_T>::Ptr& cloud2) {
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->setBackgroundColor(BG_R, BG_G, BG_B);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (BG_R, BG_G, BG_B);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v1);
    std::stringstream textstream;
    textstream << "Cloud #"<<0;
    std::stringstream ctrlstream;
    ctrlstream << "v"<<0<<"text";
    viewer->addText(textstream.str().c_str(), 10, 10, ctrlstream.str().c_str(), v1);
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height(cloud1, std::string("z"));
    std::stringstream cloudstr;
    cloudstr <<"cloud"<<0;
    viewer->addPointCloud<Point_T> (cloud1, height, cloudstr.str().c_str(), v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr.str().c_str());
    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (BG_R, BG_G, BG_B);
    //    viewer->setBackgroundColor (BG_R, BG_G, BG_B, v2);
    std::stringstream textstream2;
    textstream2 << "Cloud #"<<1;
    std::stringstream ctrlstream2;
    ctrlstream2 << "v"<<1<<"text";
    viewer->addText(textstream2.str().c_str(), 10, 10, ctrlstream2.str().c_str(), v2);
    pcl::visualization::PointCloudColorHandlerGenericField<Point_T> 
      height2(cloud2, std::string("z"));
    std::stringstream cloudstr2;
    cloudstr2 <<"cloud"<<1;
    viewer->addPointCloud<Point_T> (cloud2, height2, cloudstr2.str().c_str(), v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudstr2.str().c_str());

  }else {
    //Update
  }
  return viewer;
}
