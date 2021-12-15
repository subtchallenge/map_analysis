#include <map_analysis/mesh.h>
#include <ros/ros.h>

const aiScene* RefMesh::loadScene(const std::string& path) {
  const aiScene* scene = importer.ReadFile(path.c_str(), aiProcessPreset_TargetRealtime_MaxQuality);
  if (scene != NULL) {
    ROS_INFO_STREAM("Loaded scene!");
    ROS_INFO_STREAM("Scene has " << scene->mNumMeshes << " meshes in it ");
    size_t vert_count = 0;
    size_t face_count = 0;
    for (size_t i = 0; i < scene->mNumMeshes; i++) {
      vert_count += scene->mMeshes[i]->mNumVertices;
      face_count += scene->mMeshes[i]->mNumFaces;
    }
    ROS_INFO_STREAM("Total : " << vert_count << " vertices and " << face_count << " faces");
  } else {
    ROS_ERROR_STREAM("Did not load scene!");
    return NULL;
  }
  return scene;
}

RefMesh::RefMesh() {
  mesh_pub_in = nh.advertise<visualization_msgs::Marker>("mesh_geometry_in", 1, true);
  mesh_pub_out = nh.advertise<visualization_msgs::Marker>("mesh_geometry_out", 1, true);
}
RefMesh::~RefMesh() {
  if (vertices_flann.ptr())
    delete [] vertices_flann.ptr();
}
void PrintMat(const aiMatrix4x4& mat) {
  std::cout << mat.a1 << ", " << mat.a2 << ", " << mat.a3 << ", " << mat.a4 << std::endl;
  std::cout << mat.b1 << ", " << mat.b2 << ", " << mat.b3 << ", " << mat.b4 << std::endl;
  std::cout << mat.c1 << ", " << mat.c2 << ", " << mat.c3 << ", " << mat.c4 << std::endl;
  std::cout << mat.d1 << ", " << mat.d2 << ", " << mat.d3 << ", " << mat.d4 << std::endl;
}

void RefMesh::WalkScene(const aiNode* node, const aiMatrix4x4& transf, int depth) {
  std::stringstream preamble_ss;
  for (int i = 0; i < depth; i++)
    preamble_ss << "\t";
  std::string preamble = preamble_ss.str();
  if (node) {
    ROS_INFO_STREAM(preamble << " Node " << node->mName.C_Str() << " has " << node->mNumMeshes << " meshes and " << node->mNumChildren << " children");
    aiMatrix4x4 cur_transf = transf * node->mTransformation;
    PrintMat(cur_transf);
    if (node->mNumMeshes != 0) {
      std::vector<unsigned int> mesh_inds;
      for (int i = 0; i < node->mNumMeshes; i++) {
        mesh_inds.push_back(node->mMeshes[i]);
      }
      meshes_to_instantiate.push_back(std::make_pair(cur_transf, mesh_inds));
    }
    for (int i = 0; i < node->mNumChildren; i++) {
      WalkScene(node->mChildren[i], cur_transf, depth+1);
    }
  } else {
    ROS_INFO_STREAM(preamble << "Node is null");
  }
}
VertHolder TransformVert(const VertHolder& vh, const aiMatrix4x4& transf) {
  VertHolder vh_out = vh;
  double scale = transf.d1 + transf.d2 + transf.d3 + transf.d4;
  vh_out.vert.x = vh.vert.x * transf.a1 + vh.vert.y * transf.a2 + vh.vert.z * transf.a3 + transf.a4;
  vh_out.vert.y = vh.vert.x * transf.b1 + vh.vert.y * transf.b2 + vh.vert.z * transf.b3 + transf.b4;
  vh_out.vert.z = vh.vert.x * transf.c1 + vh.vert.y * transf.c2 + vh.vert.z * transf.c3 + transf.c4;
  vh_out.vert.x /= scale;
  vh_out.vert.y /= scale;
  vh_out.vert.z /= scale;
  return vh_out;
}
void RefMesh::Process() {
  // Pseudocode
  // 1) Walk the meshes in the scene and add their vertices to a master vertex list
  // 1.5) Need to load the SDF file and bake all transforms into model files
  // 2) Walk the faces and add to master face list
  // 3) These structures offset indices by count as we add new meshes (for master mesh union)
  // 4) Reverse the face -> vertex association and populate a vector of vertices with face refs
  //
  ROS_INFO_STREAM("Reference mesh loading");
  const aiScene*
    scene = loadScene("/home/jgrogers/Documents/satsop_alpha_mesh/meshes/satsopalpha.dae");
    //scene = loadScene("/home/jgrogers/Downloads/cave_circuit/cave_circuit_04/meshes/worldMesh.dae");
  if (scene == NULL) {
    ROS_ERROR_STREAM("No scene loaded!");
    exit(0);
  }
  ROS_INFO_STREAM("Reference mesh has finished loading");
  aiMatrix4x4 global_offset =
    aiMatrix4x4(1.0, 0, 0, 32.0,
                0, 1.0, 0, 40.0,
                0, 0, 1.0, -3.0,
                0, 0, 0, 1) *
    aiMatrix4x4(0.62, 0, 0, 0,
                0, 0.62, 0, 0,
                0, 0, 0.62, 0,
                0, 0, 0, 1) *
    aiMatrix4x4(0, -1, 0, 0,
                1, 0, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1) *
    aiMatrix4x4(1, 0, 0, 0,
                0, 0, -1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1);
  WalkScene(scene->mRootNode, global_offset);
  size_t vert_count = 0;
  size_t face_count = 0;
  ROS_INFO_STREAM("Scene still has " << scene->mNumMeshes << " meshes in it ");
  model_meshes.resize(scene->mNumMeshes);
  for (size_t i = 0; i < scene->mNumMeshes; i++) {
    ROS_INFO_STREAM("Mesh " << i << " is called " << scene->mMeshes[i]->mName.C_Str()
                    << " and has " << scene->mMeshes[i]->mNumVertices << " vertices and "
                    << scene->mMeshes[i]->mNumFaces << " faces");
    std::vector<VertHolder> tmp_vert_vec;
    for (size_t j = 0; j < scene->mMeshes[i]->mNumVertices; j ++) {
      VertHolder vh;
      vh.vert = scene->mMeshes[i]->mVertices[j];
      tmp_vert_vec.push_back(vh);
    }
    std::vector<aiFace> tmp_face_vec;
    for (size_t j = 0; j < scene->mMeshes[i]->mNumFaces; j ++) {
      tmp_face_vec.push_back(scene->mMeshes[i]->mFaces[j]);
    }
    model_meshes[i] = std::make_pair(tmp_vert_vec, tmp_face_vec);
  }
  // Now instantiate all meshes with appropriate transforms
  for (size_t i = 0; i < meshes_to_instantiate.size(); i++) {
    aiMatrix4x4 transf = meshes_to_instantiate[i].first;
    std::vector<unsigned int> meshes = meshes_to_instantiate[i].second;
    for (size_t j = 0; j < meshes.size(); j++) {
      for (size_t k = 0; k < model_meshes[meshes[j]].first.size(); k++) {
        vertices.push_back(TransformVert(model_meshes[meshes[j]].first[k], transf));
      }
      for (size_t k = 0; k < model_meshes[meshes[j]].second.size(); k++) {
        if (model_meshes[meshes[j]].second[k].mNumIndices == 3) {
          faces.push_back(model_meshes[meshes[j]].second[k]);
          for (size_t l = 0; l < 3; l++) {
            faces.back().mIndices[l] += vert_count;  // Offset into union structure
          }
          face_count++;
        }
      }
      vert_count += model_meshes[meshes[j]].first.size();
    }
  }
  membership_vector.resize(faces.size(), false);
  hit_count.resize(faces.size(), 0);
  size_t most_faces = 0;
  size_t most_verts = 0;
  aiVector3D min_pt = vertices[0].vert, max_pt = min_pt;
  for (size_t i = 0; i < vertices.size(); i++) {
    if (vertices[i].vert.x < min_pt.x) min_pt.x = vertices[i].vert.x;
    if (vertices[i].vert.y < min_pt.y) min_pt.y = vertices[i].vert.y;
    if (vertices[i].vert.z < min_pt.z) min_pt.z = vertices[i].vert.z;
    if (vertices[i].vert.x > max_pt.x) max_pt.x = vertices[i].vert.x;
    if (vertices[i].vert.y > max_pt.y) max_pt.y = vertices[i].vert.y;
    if (vertices[i].vert.z > max_pt.z) max_pt.z = vertices[i].vert.z;
  }
  for (size_t i = 0; i < faces.size(); i++) {
    if (faces[i].mNumIndices > most_verts) {
      most_verts = faces[i].mNumIndices;
      ROS_INFO_STREAM("Updated most verts to " << most_verts);
    }
    for (size_t j = 0; j < faces[i].mNumIndices; j++) {
      unsigned int vert_index = faces[i].mIndices[j];
      if (vert_index > vertices.size()) {
        ROS_ERROR_STREAM("Got vertex reference of " << vert_index << " Which is outside of vertices size " << vertices.size());
      } else {
        vertices[vert_index].face_list.push_back(i);
        if (vertices[vert_index].face_list.size() > most_faces)
          most_faces = vertices[vert_index].face_list.size();
      }
    }
  }
  std::vector<size_t> vert_histogram(most_faces+1, 0);
  std::vector<size_t> face_histogram(most_verts+1, 0);

  for (size_t i = 0; i < vertices.size(); i++) {
    vert_histogram[vertices[i].face_list.size()]++;
  }
  for (size_t i = 0; i < vert_histogram.size(); i++) {
    ROS_INFO_STREAM("Vertices with " << i << " faces : " << vert_histogram[i]);
  }
  for (size_t i = 0; i < faces.size(); i++) {
    face_histogram[faces[i].mNumIndices]++;
  }
  for (size_t i = 0; i < face_histogram.size(); i++) {
    ROS_INFO_STREAM("Faces with " << i << " vertices : " << face_histogram[i]);
  }
  ROS_INFO_STREAM("Bounding box on mesh: ("
                  << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ") to ("
                  << max_pt.x <<", " << max_pt.y << ", " << max_pt.z << ")");
  // Build the FLANN object on vertices
  ROS_INFO_STREAM("Building FLANN index of mesh vertices");
  vertices_flann = flann::Matrix<float>(new float[vertices.size()*3],
                                        vertices.size(), 3);
  for (size_t i = 0; i < vertices.size(); i++) {
    vertices_flann[i][0] = static_cast<float>(vertices[i].vert.x);
    vertices_flann[i][1] = static_cast<float>(vertices[i].vert.y);
    vertices_flann[i][2] = static_cast<float>(vertices[i].vert.z);
  }
  GenerateNormals();
  flann_index.reset(new flann::Index<flann::L2<float>>(vertices_flann,
                                                 flann::KDTreeIndexParams(4)));
  flann_index->buildIndex();
  ROS_INFO_STREAM("FLANN index of mesh vertices built");
  PublishMesh();
}
void RefMesh::GenerateNormals() {
  // Normals are defined here on faces instead of vertices
  face_normals.resize(faces.size());
  face_areas.resize(faces.size());
  for (size_t i = 0; i < faces.size(); i++) {
    // Loop over the vertices and compute displacement vectors. Normal will be the cross product of [v1-v0] x [v2-v1]
    if (faces[i].mNumIndices >= 3) {
      aiVector3D v1 = vertices[faces[i].mIndices[1]].vert - vertices[faces[i].mIndices[0]].vert;
      aiVector3D v2 = vertices[faces[i].mIndices[2]].vert - vertices[faces[i].mIndices[1]].vert;
      // i   j   k
      // x1  y1  z1
      // x2  y2  z2
      aiVector3D cross_product = aiVector3D(v1[1]*v2[2] - v1[2] * v2[1],
                                            -v1[0]*v2[2] + v1[2]* v2[0],
                                            v1[0]*v2[1] - v1[1]*v2[0]);
      face_areas[i] = cross_product.Length() / 2.0;
      
      face_normals[i] = cross_product.Normalize();
    } else {
      face_normals[i] = aiVector3D(0.0, 0.0, 0.0);
      face_areas[i] = 0.0;
    }
  }
}

void RefMesh::PublishMesh() {
  visualization_msgs::Marker msg;
  msg.header.stamp = ros::Time(0);
  msg.header.frame_id = "darpa";
  msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = 1.0;
  msg.scale.y = 1.0;
  msg.scale.z = 1.0;
  visualization_msgs::Marker msg_out = msg;
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 0.5;
  msg_out.color.r = 1.0;
  msg_out.color.g = 0.0;
  msg_out.color.b = 0.0;
  msg_out.color.a = 0.5;
  for (size_t i = 0; i < faces.size(); i++) {
    if (faces[i].mNumIndices == 3) {
      geometry_msgs::Point pt1, pt2, pt3;
      pt1.x = vertices[faces[i].mIndices[0]].vert.x;
      pt1.y = vertices[faces[i].mIndices[0]].vert.y;
      pt1.z = vertices[faces[i].mIndices[0]].vert.z;
      pt2.x = vertices[faces[i].mIndices[1]].vert.x;
      pt2.y = vertices[faces[i].mIndices[1]].vert.y;
      pt2.z = vertices[faces[i].mIndices[1]].vert.z;
      pt3.x = vertices[faces[i].mIndices[2]].vert.x;
      pt3.y = vertices[faces[i].mIndices[2]].vert.y;
      pt3.z = vertices[faces[i].mIndices[2]].vert.z;
      if (membership_vector[i]) {
        msg.points.push_back(pt1);
        msg.points.push_back(pt2);
        msg.points.push_back(pt3);
      } else {
        msg_out.points.push_back(pt1);
        msg_out.points.push_back(pt2);
        msg_out.points.push_back(pt3);
      }
    }
  }
  mesh_pub_in.publish(msg);
  mesh_pub_out.publish(msg_out);
}

std::tuple<double, double> RefMesh::
DistCheck(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  // Perform a FLANN lookup for the closest N>3 mesh vertices
  // Find the face(s) which are most in common with vertices
  //
  ros::Time mesh_start_time = ros::Time::now();
  for (size_t i = 0; i < membership_vector.size(); i++) {
    membership_vector[i] = false;
    hit_count[i] = 0;
  }
  ROS_INFO_STREAM("Preparing point cloud for vertex FLANN query");
  size_t nn = 6;
  flann::Matrix<float> query(new float[cloud.points.size()* 3],
                             cloud.points.size(), 3);
  for (size_t i = 0; i < cloud.points.size(); i++) {
    query[i][0] = static_cast<float>(cloud.points[i].x);
    query[i][1] = static_cast<float>(cloud.points[i].y);
    query[i][2] = static_cast<float>(cloud.points[i].z);
  }
  flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
  flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
  flann_index->knnSearch(query, indices, dists, nn, flann::SearchParams(32));
  ROS_INFO_STREAM("Vertex FLANN query completed");
  // Now lets go over each point and check the vertices in the closest list
  for (size_t i = 0; i < query.rows; i++) {
    std::map<size_t, size_t> face_candidates;
    for (size_t j = 0; j < nn; j++) {
      int candidate = indices[i][j];
      for (std::list<unsigned int>::const_iterator itr = vertices[candidate].face_list.begin();
           itr != vertices[candidate].face_list.end(); itr++) {
        if (face_candidates.find(*itr) == face_candidates.end()) {
          face_candidates[*itr] = 1;
        } else {
          face_candidates[*itr]++;
        }
      }
    }
    // Not sure if multiplicity actually matters above -- I think we have to check all
    // candidate faces and apply this to the closest one with an internal projection...
    //
    float min_dist = std::numeric_limits<float>::infinity();
    int best_face = -1;
    for (std::map<size_t, size_t>::const_iterator itr = face_candidates.begin();
         itr != face_candidates.end(); itr++) {
      size_t face = itr->first;
      // Test this point for distance and internal projection
      aiVector3D testpt(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      std::pair<float, aiVector3D> projection =
        ProjectedPointInPlane(face_normals[face],
                              vertices[faces[face].mIndices[0]].vert,
                              testpt);
      if (projection.first > min_dist) continue; // Further than best, doesn't matter if internal
      if (InsideTriangle(projection.second,
                         vertices[faces[face].mIndices[0]].vert,
                         vertices[faces[face].mIndices[1]].vert,
                         vertices[faces[face].mIndices[2]].vert)) {
        min_dist = projection.first;
        best_face = face;
      }
    }
    if (best_face != -1) {
      membership_vector[best_face] = true;
      hit_count[best_face]++;
    }
  }
  double point_count = 0.0;
  double per_square_meter = 0.0;
  double area_total = 0;
  for (size_t i = 0; i < hit_count.size(); i++) {
    area_total += face_areas[i];
    if (hit_count[i] != 0) {
      point_count += hit_count[i];
      per_square_meter += face_areas[i];
    }
  }
  double ppsm = point_count / per_square_meter;
  double mesh_area_ratio = per_square_meter / area_total;

  delete[] query.ptr();
  delete[] indices.ptr();
  delete[] dists.ptr();
  ROS_INFO_STREAM("Elapsed time by cloud mesh analysis: " << (ros::Time::now() - mesh_start_time).toSec());
  return std::make_tuple(ppsm, mesh_area_ratio);
}
