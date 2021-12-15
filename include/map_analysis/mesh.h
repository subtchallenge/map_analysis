// Mesh import stuff
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <vector>
#include <map>
#include <list>
#include <utility>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <flann/flann.hpp>

const aiScene* loadScene(const std::string& path);
class VertHolder {

 public:
  VertHolder() {}
  aiVector3D vert;
  std::list<unsigned int> face_list;
};
class RefMesh {
 protected:
  std::vector<aiFace> faces;
  std::vector<VertHolder> vertices;
  std::vector<aiVector3D> face_normals;
  std::vector<std::pair<std::vector<VertHolder>, std::vector<aiFace>>> model_meshes;
  std::vector<bool> membership_vector;
  std::vector<size_t> hit_count;
  std::vector<double> face_areas;
  Assimp::Importer importer;
  boost::shared_ptr<flann::Index<flann::L2<float>>> flann_index;
  flann::Matrix<float> vertices_flann;
  const aiScene* loadScene(const std::string& path);
  ros::Publisher mesh_pub_in;
  ros::Publisher mesh_pub_out;
  ros::NodeHandle nh;
  std::vector<std::pair<aiMatrix4x4, std::vector<unsigned int>>> meshes_to_instantiate;
  void WalkScene(const aiNode* node, const aiMatrix4x4& transf = aiMatrix4x4(), int depth = 0);
  void GenerateNormals();

 public:
  RefMesh();
  virtual ~RefMesh();
  void Process();
  void PublishMesh();
  // Returns the distance to the nearest face in the mesh along with the face index
  std::tuple<double, double> DistCheck(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  aiVector3D CrossProduct(const aiVector3D& a, const aiVector3D& b) const {
    return aiVector3D(a[1]*b[2] - a[2]*b[1],
                      -a[0]*b[2] + a[2]*b[0],
                      a[0]*b[1] - a[1]*b[0]);
  }

  float DotProduct(const aiVector3D& a, const aiVector3D& b) const {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  }

  bool SameSide(const aiVector3D& p1, const aiVector3D& p2, const aiVector3D& a, const aiVector3D&b) const {
    aiVector3D cp1 = CrossProduct(b-a, p1-a);
    aiVector3D cp2 = CrossProduct(b-a, p2-a);
    if (DotProduct(cp1, cp2) >= 0) {
      return true;
    } else {
      return false;
    }
  }
  bool InsideTriangle(const aiVector3D& p, const aiVector3D& a, const aiVector3D& b, const aiVector3D& c) const {
    return SameSide(p, a, b, c) && SameSide(p, b, c, a) && SameSide(p, c, a, b);
  }
  std::pair<float, aiVector3D> ProjectedPointInPlane(const aiVector3D& normal, const aiVector3D& pip, const aiVector3D& testpt) const {
    // Finds the point in the plane which is closest to this point
    // All points within the plane (x,y,z) given a point in the plane (x0,y0,z0) and normal (a,b,c)
    // will satisfy a(x-x0) + b(y-y0) + c(z-z0) = 0
    aiVector3D v = testpt - pip;
    float d = normal[0] * v[0] + normal[1] * v[1] + normal[2] * v[2];
    aiVector3D projpt = testpt + d * normal;
    return std::make_pair(fabs(d), projpt);
  }
};
