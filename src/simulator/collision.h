//
// Created by ZHOUYC on 2022/8/5.
//



#ifndef ROBOT_OCC_COLLISION_H
#define ROBOT_OCC_COLLISION_H
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "fcl/narrowphase/collision_object.h"
#include "fcl/config.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/bvh/BVH_utility.h"
#include "fcl/math/bv/AABB.h"
#include "fcl/math/bv/utility.h"
#include <fcl/math/bv/OBBRSS.h>
#include <iostream>
#include <Eigen/Dense>


auto collision()->void;
void createMesh();


#endif //ROBOT_OCC_COLLISION_H
