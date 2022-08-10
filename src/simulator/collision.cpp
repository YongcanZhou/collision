//
// Created by ZHOUYC on 2022/8/5.
//

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "fcl/fcl.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/config.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/math/bv/AABB.h"
#include <fcl/math/bv/OBBRSS.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

#pragma comment (lib, "assimp-vc142-mt.lib")
#pragma comment (lib, "fcl.lib")
using namespace std;
using namespace fcl;
using std::map;
using std::pair;
using std::string;
using std::vector;
using fcl::AngleAxis;
using fcl::Transform3;
using fcl::Vector3;


//==============================================================================
template <class BV>
fcl::BVHModel<BV>* createMesh(const aiScene* _mesh)
{
  // Create FCL mesh from Assimp mesh
  assert(_mesh);
//      typedef BVHModel<OBB<double>> Model;
//      std::shared_ptr<BVHModel<BV> > geom(new BVHModel<BV>);
//      shared_ptr<BV> geom = std::make_shared<BV>();
//  auto geom = new BVHModel<BV>;
//  typedef BVHModel<OBBRSSd> Model;
//  auto geom = new BVHModel<OBBRSS<double>>;
  auto geom = new fcl::BVHModel<BV>;
  geom->beginModel();
  for (std::size_t i = 0; i < _mesh->mNumMeshes; i++)
  {
    for (std::size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++)
    {
      fcl::Vector3d vertices_fcl[3];
      for (std::size_t k = 0; k < 3; k++)
      {
        const aiVector3D& vertex = _mesh->mMeshes[i]->mVertices[_mesh->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices_fcl[k] = fcl::Vector3d (vertex.x, vertex.y, vertex.z);
      }
      geom->addTriangle(vertices_fcl[0], vertices_fcl[1], vertices_fcl[2]);
    }
  }
  geom->endModel();
  geom->computeLocalAABB();
  return geom;
}

void LoadFinish(const aiScene* scene)
{

  std::cout << "LoadFinish ! NumVertices : " << (*(scene->mMeshes))->mNumVertices << std::endl;
}

bool LoadModel(const std::string& pFile)
{
  // Create an instance of the Importer class
  Assimp::Importer importer;

  // And have it read the given file with some example postprocessing
  // Usually - if speed is not the most important aspect for you - you'll
  // probably to request more postprocessing than we do in this example.
  const aiScene* scene = importer.ReadFile(pFile,
                                           aiProcess_CalcTangentSpace |
                                           aiProcess_Triangulate |
                                           aiProcess_JoinIdenticalVertices |
                                           aiProcess_SortByPType);

  // If the import failed, report it
  if (!scene)
  {
    std::cout << importer.GetErrorString() << std::endl;
    return false;
  }

  // Now we can access the file's contents.
  LoadFinish(scene);
//  fcl::CollisionGeometry* geom = nullptr;
  auto geom = createMesh<OBBRSS<double>>(scene);

//      BVHModel<OBBRSSd> geom = createMesh<OBBRSSd>(scene);
//      scene->mRootNode   aiNode *node

  // We're done. Everything will be cleaned up by the importer destructor
  return true;
}

auto collision()->void
{
  LoadModel(R"(C:\Users\ZHOUYC\Desktop\contact\3_collision\collision\ee.STL)");
}




