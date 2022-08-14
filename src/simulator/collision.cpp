//
// Created by ZHOUYC on 2022/8/5.
//
#include <memory>
#include <src/simulator/collision.h>

using namespace std;
using namespace fcl;
using std::map;
using std::pair;
using std::string;
using std::vector;
using fcl::AngleAxis;
using fcl::Transform3;
using fcl::Vector3;

namespace zyc{

  Loader::Loader() : importer(new Assimp::Importer()) {
    // set list of ignored parameters (parameters used for rendering)
    importer->SetPropertyInteger(
            AI_CONFIG_PP_RVC_FLAGS,
            aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
            aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
            aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_TEXTURES |
            aiComponent_TEXCOORDS | aiComponent_MATERIALS | aiComponent_NORMALS);

    // remove LINES and POINTS
    importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                                 aiPrimitiveType_LINE | aiPrimitiveType_POINT);
  }

  Loader::~Loader() {
    if (importer) delete importer;
  }

  void Loader::load(const std::string& resource_path) {
    scene = importer->ReadFile(
            resource_path.c_str(),
            aiProcess_SortByPType | aiProcess_Triangulate |
            aiProcess_RemoveComponent | aiProcess_ImproveCacheLocality |
            aiProcess_FindDegenerates | aiProcess_JoinIdenticalVertices);

    if (!scene) {
      const std::string exception_message(
              std::string("Could not load resource ") + resource_path +
              std::string("\n") + importer->GetErrorString() + std::string("\n") +
              "Hint: the mesh directory may be wrong.");
      throw std::invalid_argument(exception_message);
    }

    if (!scene->HasMeshes())
      throw std::invalid_argument(std::string("No meshes found in file ") +
                                  resource_path);
  }

  unsigned recurseBuildMesh(const fcl::Vector3f& scale, const aiScene* scene,
                            const aiNode* node, unsigned vertices_offset,
                            TriangleAndVertices& tv) {
    if (!node) return 0;

    aiMatrix4x4 transform = node->mTransformation;
    aiNode* pnode = node->mParent;
    while (pnode) {
      // Don't convert to y-up orientation, which is what the root node in
      // Assimp does
      if (pnode->mParent != nullptr) {
        transform = pnode->mTransformation * transform;
      }
      pnode = pnode->mParent;
    }

    unsigned nbVertices = 0;
    for (uint32_t i = 0; i < node->mNumMeshes; i++) {
      aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

      // Add the vertices
      for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
        aiVector3D p = input_mesh->mVertices[j];
        p *= transform;
        tv.vertices_.push_back(
                fcl::Vector3f (p.x * scale[0], p.y * scale[1], p.z * scale[2]));
      }

      cout<<"mNumFaces"<<":"<<input_mesh->mNumFaces<<endl;

      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
        aiFace& face = input_mesh->mFaces[j];
        assert(face.mNumIndices == 3 && "The size of the face is not valid.");

        tv.triangles_.push_back(
                fcl::Triangle(vertices_offset + face.mIndices[0],
                              vertices_offset + face.mIndices[1],
                              vertices_offset + face.mIndices[2]));
      }
      cout<<"mNumVertices"<<":"<<input_mesh->mNumVertices<<endl;
      nbVertices += input_mesh->mNumVertices;
    }

    for (uint32_t i = 0; i < node->mNumChildren; ++i) {
      nbVertices += recurseBuildMesh(scale, scene, node->mChildren[i], nbVertices, tv);
    }

    return nbVertices;

  }

  void buildMesh(const fcl::Vector3f& scale, const aiScene* scene,
                 unsigned vertices_offset, TriangleAndVertices& tv) {
    recurseBuildMesh(scale, scene, scene->mRootNode, vertices_offset, tv);
  }

  void meshFromAssimpScene(
    const fcl::Vector3f & scale, const aiScene* scene,
    const shared_ptr<fcl::BVHModel<OBBRSSf> >& mesh) {
    TriangleAndVertices tv;

    mesh->beginModel();

    buildMesh(scale, scene, (unsigned)mesh->num_vertices, tv);
    mesh->addSubModel(tv.vertices_, tv.triangles_);

    mesh->endModel();
  }

  void fclCollision() {

  Loader scene;
  scene.load("C://Users//ZHOUYC//Desktop//contact//3_collision//collision//ee.stl");
//  scene.load("D:/app/assimp/test/models/STL/triangle_with_two_solids.stl");
  typedef fcl::BVHModel<OBBRSSf> Model;
  std::shared_ptr<Model> geom = std::make_shared<Model>();
  Vector3f scale{1,1,1};
  meshFromAssimpScene(scale, scene.scene, geom);
  geom->computeLocalAABB();
  cout<<"geom.get().aabb_center: "<<geom.get()->aabb_center[0]<<" "<< geom.get()->aabb_center[1]<<" "<<geom.get()->aabb_center[2]
  <<" geom.get.aabb_radius: "<<geom.get()->aabb_radius<<endl;
  fcl::Transform3<float> X_WBV = fcl::Transform3<float>::Identity();
//  X_WBV.translation() << -365, 0, -642;
  X_WBV.translation() << 0,0, 0;
//  X_WBV.linear() << 0,0,1, 0,1,0, -1,0,0;//y axis 90
  auto Robot = new CollisionObjectf(geom, X_WBV);

//  cout<<"Configure sphere geometry."<<endl;
//  // Configure sphere geometry.
//  using Real = typename fcl::constants<float>::Real;
//  const Real r = 30;
//  auto sphere_geometry = std::make_shared<fcl::Sphere<float>>(r);
//  // Poses of the geometry.
//  fcl::Transform3<float> X_WS = fcl::Transform3<float>::Identity();
//  X_WS.translation() << 0, 0, 0;
//  fcl::CollisionObject<float> sphere(sphere_geometry, X_WS);

    Loader scene_sphere;
    scene_sphere.load("C://Users//ZHOUYC//Desktop//contact//3_collision//collision//sphere.stl");
//  scene.load("D:/app/assimp/test/models/STL/triangle_with_two_solids.stl");
    typedef fcl::BVHModel<OBBRSSf> Model;
    std::shared_ptr<Model> geom_sphere = std::make_shared<Model>();
    Vector3f scale_sphere{1,1,1};
    meshFromAssimpScene(scale_sphere, scene_sphere.scene, geom_sphere);
    geom_sphere->computeLocalAABB();
    cout<<"geom.get().aabb_center: "<<geom_sphere.get()->aabb_center[0]<<" "<< geom_sphere.get()->aabb_center[1]<<" "<<geom_sphere.get()->aabb_center[2]
        <<" geom.get.aabb_radius: "<<geom_sphere.get()->aabb_radius<<endl;
    fcl::Transform3<float> X_WS = fcl::Transform3<float>::Identity();
//  X_WS.translation() << -365, 0, -642;
    X_WS.translation() << -155, 0, 0;
//    X_WS.linear() << 0,0,1, 0,1,0, -1,0,0;//y axis 90
    auto Sphere = new CollisionObjectf(geom, X_WBV);

  cout<<"Compute collision"<<endl;
  // Compute collision - single contact and enable contact.
  fcl::CollisionRequest<float> Request(1, true);
  // Request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  fcl::CollisionResult<float> Result;
  std::size_t num_contacts = fcl::collide(Sphere, Robot, Request, Result);//collision
  std::vector<fcl::Contact<float>> contacts;
  Result.getContacts(contacts);
  cout << "num "<<contacts.size() << " contacts found" << endl;
  for(const Contact<float> &contact : contacts) {
    cout << "position: " << contact.pos << endl;
  }

  // set the distance request structure, here we just use the default setting
    DistanceRequest<float> request;
  // result will be returned via the collision result structure
    DistanceResult<float> result;
  // perform distance test
    distance(Sphere, Robot, request, result);
  cout<<result.min_distance<<endl;

  delete Robot;
  delete Sphere;

  }

}//namespace zyc
