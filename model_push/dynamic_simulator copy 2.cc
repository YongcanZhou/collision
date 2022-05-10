
#include <iostream>
#include <thread>
#include <mutex>

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

#include "dynamic_simulator.h"

using namespace std;
using std::map;
using std::pair;
using std::string;
using std::vector;

namespace zyc
{
  
  std::thread sim_thread_;  
  std::mutex sim_mutex_;

  // Simple specification for defining a box collision object. 
  // Specifies the dimensions and pose of the box in some frame F (X_FB).
  // X_FB is the "pose" of frame B in frame F.
  struct BoxSpecification {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fcl::Vector3<double> size;
    fcl::Transform3<double> X_FB;
  };

  // Class for executing and evaluating various box-box tests.
  // The class is initialized with two box specifications (size and pose).
  // The test performs a collision test between the two boxes in 12 permutations(排列):
  // One axis is the order (box 1 vs box 2 and box 2 vs box 1).
  // The other axis is the orientation box 2. Given that box 2 must be a cube, it
  // can be oriented in six different configurations and still produced the same
  // answer.
  // The 12 permutations are the two orderings crossed with the six orientations.
  
  class BoxBoxTest {
  public:
    // Construct the test scenario with the given box specifications. Box 2
    // must be a *cube* (all sides equal).
    BoxBoxTest(const BoxSpecification box_spec_1,
                const BoxSpecification box_spec_2)
        : box_spec_1_(box_spec_1), box_spec_2_(box_spec_2) {
      using fcl::AngleAxis;
      using fcl::Transform3;
      using fcl::Vector3;

      // Confirm box 2 is a cube
      //  EXPECT_EQ(box_spec_2.size(0), box_spec_2.size(1));
      //  EXPECT_EQ(box_spec_2.size(0), box_spec_2.size(2));

      const double pi = fcl::constants<double>::pi();

      // Initialize isomorphic rotations of box 2.
      iso_poses_["top"] = Transform3<double>::Identity();
      iso_poses_["bottom"] =
          Transform3<double>{AngleAxis<double>(pi, Vector3<double>::UnitX())};
      iso_poses_["back"] =
          Transform3<double>{AngleAxis<double>(pi / 2, Vector3<double>::UnitX())};
      iso_poses_["front"] =
          Transform3<double>{AngleAxis<double>(3 * pi / 2, Vector3<double>::UnitX())};
      iso_poses_["left"] =
          Transform3<double>{AngleAxis<double>(pi / 2, Vector3<double>::UnitY())};
      iso_poses_["right"] =
          Transform3<double>{AngleAxis<double>(3 * pi / 2, Vector3<double>::UnitY())};
    }

    // Runs the 12 tests for the two specified boxes.
    //
    // @param solver_type       The solver type to use for computing collision.
    // @param test_tolerance    The tolerance to which the collision contact
    //                          results will be compared to the results.
    // @param expected_normal   The expected normal for the (1, 2) query order.
    // @param expected_depth    The expected penetration depth.
    // @param contact_pos_test  A function to evaluate the reported contact
    //                          position for validity; this should be written to
    //                          account for the possibility of the contact
    //                          position lying on some manifold (e.g., an edge, or
    //                          face). This function should invoke googletest
    //                          EXPECT_* methods.
    // @param origin_name       A string which is appended to error message to
    //                          more easily parse failures and which test failed.
    // void
    //  (fcl::GJKSolverType solver_type, S test_tolerance,
    //          const fcl::Vector3<S>& expected_normal, S expected_depth,
    //          std::function<void(const fcl::Vector3<S> &, S, const std::string &)>
    //              contact_pos_test,
    //          const std::string& origin_name) {
    //   fcl::Contact<S> expected_contact;
    //   expected_contact.penetration_depth = expected_depth;

    //   for (const auto& reorient_pair : iso_poses_) {
    //     const std::string& top_face = reorient_pair.first;
    //     const fcl::Transform3<S>& pre_pose = reorient_pair.second;

    //     BoxSpecification<S> box_2_posed{
    //         box_spec_2_.size,
    //         box_spec_2_.X_FB * pre_pose
    //     };

    //     // Collide (1, 2)
    //     expected_contact.normal = expected_normal;
    //     RunSingleTest(box_spec_1_,
    //                   box_2_posed,
    //                   solver_type,
    //                   test_tolerance,
    //                   expected_contact,
    //                   contact_pos_test,
    //                   origin_name + " (1, 2) - " + top_face);

    //     // Collide (2, 1)
    //     expected_contact.normal = -expected_normal;
    //     RunSingleTest(box_2_posed,
    //                   box_spec_1_,
    //                   solver_type,
    //                   test_tolerance,
    //                   expected_contact,
    //                   contact_pos_test,
    //                   origin_name + " (2, 1) - " + top_face);
    //   }
    // }

  private:
  // Performs a collision test between two boxes and tests the *single* contact
  // result against given expectations.
  //
  // @param box_spec_A        A specification of the first box (treated as object
  //                          1 in the query).
  // @param box_spec_B        A specification of the second box (treated as object
  //                          2 in the query).
  // @param solver_type       The solver type to use for computing collision.
  // @param test_tolerance    The tolerance to which the collision contact results
  //                          will be compared to the results.
  // @param expected_contact  The expected contact details (only penetration depth
  //                          and normal are used).
  // @param contact_pos_test  A function to evaluate the reported contact position
  //                          for validity; this should be written to account for
  //                          the possibility of the contact position lying on
  //                          some manifold (e.g., an edge, or face). This
  //                          function should invoke googletest EXPECT_* methods.
  // @param origin_name       A string which is appended to error message to more
  //                          easily parse failures and which test failed.
    void RunSingleTest(
        const BoxSpecification box_spec_A,
        const BoxSpecification box_spec_B, fcl::GJKSolverType solver_type,
        double test_tolerance, const fcl::Contact<double>& expected_contact,
        std::function<void(const fcl::Vector3<double>&, double, const std::string&)>
        // contact_pos_test,
        // const std::string& origin_name
        ) {
      using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;
      CollisionGeometryPtr_t box_geometry_A(new fcl::Box<double>(box_spec_A.size));
      CollisionGeometryPtr_t box_geometry_B(new fcl::Box<double>(box_spec_B.size));

      fcl::CollisionObject<double> box_A(box_geometry_A, box_spec_A.X_FB);
      fcl::CollisionObject<double> box_B(box_geometry_B, box_spec_B.X_FB);

      // Compute collision - single contact and enable contact.
      fcl::CollisionRequest<double> collisionRequest(1, true);
      collisionRequest.gjk_solver_type = solver_type;
      fcl::CollisionResult<double> collisionResult;
      int num_contacts = fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);
      std::cout <<"num_contacts"<<num_contacts<<std::endl;
      // EXPECT_TRUE(collisionResult.isCollision()) << origin_name;
      std::vector<fcl::Contact<double>> contacts;
      collisionResult.getContacts(contacts);
      // GTEST_ASSERT_EQ(contacts.size(), 1u) << origin_name;

      const fcl::Contact<double>& contact = contacts[0];
      // EXPECT_NEAR(expected_contact.penetration_depth, contact.penetration_depth,
      //             test_tolerance)
      //           << origin_name;
      // EXPECT_TRUE(expected_contact.normal.isApprox(contact.normal,
      //                                              test_tolerance))
      //           << origin_name << ":\n\texpected: "
      //           << expected_contact.normal.transpose()
      //           << "\n\tcontact.normal: " << contact.normal.transpose();
      // contact_pos_test(contact.pos, test_tolerance, origin_name);
    }

    const BoxSpecification box_spec_1_;
    const BoxSpecification box_spec_2_;
    map<string, fcl::Transform3<double>, std::less<string>,
        Eigen::aligned_allocator<std::pair<const string, fcl::Transform3<double>>>>
        iso_poses_;
  };

  static double x{0.0}, y{0.0}, z{10.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, 
                duration{0.0}, delta_t{0.001}, g{9.8}, delta_z{0.0}, v_0{0.0};
  auto start_first = std::chrono::steady_clock::now();

  
  void collision_box_box(fcl::GJKSolverType solver_type,
                                          double test_tolerance)
  {
    const double pi = fcl::constants<double>::pi();
    const double size_1 = 1;
    BoxSpecification box_1{fcl::Vector3<double>{size_1, size_1, size_1},
                              fcl::Transform3<double>::Identity()};

    const double size_2 = 1;
    BoxSpecification box_2{fcl::Vector3<double>{size_2, size_2, size_2},
                              fcl::Transform3<double>::Identity()};
    box_2.X_FB.translation() << 0, 0, 10;

    fcl::Vector3<double> expected_normal{0, 0, -1};
    double expected_depth = size_1 * sqrt(2) / 2;

    // auto contact_pos_test = [size_1](const fcl::Vector3<S> &pos, S tolerance,
    //                                  const std::string& origin_name) {
    //   const double expected_pos_z = -size_1 * std::sqrt(2) / 4;
    //   EXPECT_NEAR(expected_pos_z, pos(2), tolerance) << origin_name;
    //   EXPECT_NEAR(0, pos(0), tolerance) << origin_name;
    //   EXPECT_LE(pos(1), 0.5) << origin_name;
    //   EXPECT_GE(pos(1), -0.5) << origin_name;
    // };

    // BoxBoxTest tests(box_1, box_2);
    // tests.RunTests(solver_type, test_tolerance, expected_normal, expected_depth);
                  //  contact_pos_test, "test_colliion_box_box_all_contacts"     
  }
  // collision_box_box(fcl::GJKSolverType::GST_LIBCCD,1e-14);

  auto sim_thread_fun()->void{
    while (true){
      // TODO: fcl

      auto start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
        x += 0;
        y += 0;
        duration += delta_t;
        v_0 = g*duration;
        delta_z = v_0*delta_t+1/2*g*pow(delta_t,2);
        z -= 0.01*(delta_z);//slow 100X
        roll += 0;
        pitch += 0;
        yaw += 0;
      }
      // per 1ms
      // std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
      std::this_thread::sleep_until(start + std::chrono::nanoseconds(1000000));
      //verify time 
      // auto duration = chrono::duration_cast<chrono::milliseconds>(std::chrono::steady_clock::now() - start);
      // cout << "Operation duration : " << duration.count() << "ms" << endl;
    }
  };

  auto init_simulator()->void{
    if  ( sim_thread_.joinable()){
      return;
    }
    else{
      sim_thread_ = std::thread(sim_thread_fun); 
    }
  }

  void DynamicSimulator(double &x_, double &y_,double &z_,double &roll_,double &pitch_,double  &yaw_)
  {    
    std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;    
  }
}

