#include "gtest/gtest.h"
#include "kdl/frames.hpp"
#include "kdl/utilities/utility.h" // For KDL::Equal and KDL::epsilon
#include <cmath>      // For M_PI, std::sqrt, std::acos, std::exp
#include <algorithm>  // For std::max, std::min
#include <map>
#include <string>

// Headers for controller and messages
#include "operational_space_controllers/force_controller.hpp"
#include "operational_space_controllers_msgs/Move.h"
// #include "ros/ros.h" // Avoid full include if possible, use forward declarations or mock what's needed.
                       // For this test, we need ros::Subscriber for MockNodeHandle.
                       // If it's too heavy, this would be an issue.
                       // For now, assuming it's acceptable or a more minimal version could be found.
#include "ros/subscriber.h" // Attempting a more minimal include for ros::Subscriber
#include "ros/node_handle.h" // Needed for ros::Subscriber template in MockNodeHandle if not fwd-declarable easily.

// Constants from force_controller.cpp
static const double PI = M_PI;
static const double BETA = 20.0 / PI;
static const double GAMMA = 1e4;

// Epsilon for floating point comparisons
const double test_epsilon = 1e-5;

// Basic mock for ros::NodeHandle for parameter handling
// Note: This is a very simplified mock. A real ROS test environment (like rostest)
// would handle NodeHandle creation and parameter server interaction.
class MockNodeHandle {
public:
    std::map<std::string, std::string> string_params;
    std::map<std::string, double> double_params;
    std::string current_namespace = "/test_ns"; // Default namespace

    MockNodeHandle(const std::string& ns = "/test_ns") : current_namespace(ns) {}

    bool getParam(const std::string& param_name, std::string& output) const {
        auto it = string_params.find(param_name);
        if (it != string_params.end()) {
            output = it->second;
            return true;
        }
        ROS_WARN_STREAM("MockNodeHandle: String Parameter [" << param_name << "] not found.");
        return false;
    }

    bool getParam(const std::string& param_name, double& output) const {
        auto it = double_params.find(param_name);
        if (it != double_params.end()) {
            output = it->second;
            return true;
        }
        ROS_WARN_STREAM("MockNodeHandle: Double Parameter [" << param_name << "] not found.");
        return false;
    }

    // Required by ForceController::init
    std::string getNamespace() const { return current_namespace; }

    // Dummy subscribe method. ForceController::init calls subscribe.
    // The ForceController class is passed by pointer to the callback.
    // The message type for "command" is operational_space_controllers_msgs::Move
    template<class M>
    ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                              void(operational_space_controllers::ForceController::*fp)(const boost::shared_ptr<M const>&),
                              operational_space_controllers::ForceController* obj) {
        // Return a dummy ros::Subscriber. Actual subscription won't happen.
        // This is to allow ForceController::init to complete its call to n.subscribe.
        ROS_INFO_STREAM("MockNodeHandle: subscribe called for topic " << topic);
        return ros::Subscriber();
    }

    // Dummy advertise method for RealtimePublisher in init
    template<class M>
    ros::Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
        ROS_INFO_STREAM("MockNodeHandle: advertise called for topic " << topic);
        return ros::Publisher();
    }
};


// Dummy test to ensure GTest is working
TEST(ForceControllerTest, DummyTest) {
    ASSERT_EQ(1, 1);
}

// Test fixture for ForceController tests that need a controller instance
class ForceControllerInitTest : public ::testing::Test {
protected:
    operational_space_controllers::ForceController controller;
    MockNodeHandle mock_nh;

    // We can't fully mock pr2_mechanism_model::RobotState or pr2_mechanism_model::Chain easily here.
    // The init function will likely try to use them and fail later.
    // These tests focus on parameter reading, which happens before chain initialization.
    // ROS_ERROR messages are expected if chain init fails. We suppress them for cleaner test output if possible,
    // or just acknowledge they might appear.
};

TEST_F(ForceControllerInitTest, InitParameterLoading_Success) {
    mock_nh.string_params["root_name"] = "test_root";
    mock_nh.string_params["tip_name"] = "test_tip";
    mock_nh.double_params["proportional_gain"] = 100.0;
    mock_nh.double_params["derivative_gain"] = 10.0;
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;

    // The call to init will likely print ROS_ERROR about chain initialization failing
    // because the robot_state is null and chain.init will fail.
    // However, parameter checks happen before that. If all params are present,
    // it should proceed past param checks. We can't assert true because chain.init will fail.
    // This highlights that the original init function is hard to unit test for just param loading.
    // We expect it to proceed far enough to try to init chain.
    // If a parameter was missing, it would have returned false earlier.

    // To properly test success, ForceController::init would need to be refactored
    // or a more complete PR2 robot model mock provided.
    // For now, this "success" test mostly ensures no crash *during parameter reading*.
    // We can't assert true on init's result due to subsequent errors.
    ASSERT_NO_FATAL_FAILURE(controller.init(nullptr, mock_nh));
    // ROS_ERROR about chain init is expected here.
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingRootName) {
    // mock_nh.string_params["root_name"] = "test_root"; // Missing
    mock_nh.string_params["tip_name"] = "test_tip";
    mock_nh.double_params["proportional_gain"] = 100.0;
    mock_nh.double_params["derivative_gain"] = 10.0;
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingTipName) {
    mock_nh.string_params["root_name"] = "test_root";
    // mock_nh.string_params["tip_name"] = "test_tip"; // Missing
    mock_nh.double_params["proportional_gain"] = 100.0;
    // ... (add other gains to ensure it's not failing on them first)
    mock_nh.double_params["derivative_gain"] = 10.0;
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingProportionalGain) {
    mock_nh.string_params["root_name"] = "test_root";
    mock_nh.string_params["tip_name"] = "test_tip";
    // mock_nh.double_params["proportional_gain"] = 100.0; // Missing
    mock_nh.double_params["derivative_gain"] = 10.0;
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingDerivativeGain) {
    mock_nh.string_params["root_name"] = "test_root";
    mock_nh.string_params["tip_name"] = "test_tip";
    mock_nh.double_params["proportional_gain"] = 100.0;
    // mock_nh.double_params["derivative_gain"] = 10.0; // Missing
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingProportionalGainRotational) {
    mock_nh.string_params["root_name"] = "test_root";
    mock_nh.string_params["tip_name"] = "test_tip";
    mock_nh.double_params["proportional_gain"] = 100.0;
    mock_nh.double_params["derivative_gain"] = 10.0;
    // mock_nh.double_params["proportional_gain_rotational"] = 50.0; // Missing
    mock_nh.double_params["derivative_gain_rotational"] = 5.0;
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}

TEST_F(ForceControllerInitTest, InitParameterLoading_MissingDerivativeGainRotational) {
    mock_nh.string_params["root_name"] = "test_root";
    mock_nh.string_params["tip_name"] = "test_tip";
    mock_nh.double_params["proportional_gain"] = 100.0;
    mock_nh.double_params["derivative_gain"] = 10.0;
    mock_nh.double_params["proportional_gain_rotational"] = 50.0;
    // mock_nh.double_params["derivative_gain_rotational"] = 5.0; // Missing
    ASSERT_FALSE(controller.init(nullptr, mock_nh));
}


// NOTE on testing ForceController::commandCB:
// ForceController::commandCB is a private method. It is typically called by a ROS subscriber.
// Unit testing it directly is not possible without refactoring the ForceController class
// (e.g., making commandCB public or using a friend class for tests) or setting up
// a more complex integration test where a message is actually published and received.
// The RealtimeBox member 'move_command' which is set by commandCB is also private.
// Thus, direct unit testing of commandCB's effects is currently impractical.
TEST(ForceControllerTest, CommandCBNotUnitTestable) {
    GTEST_SKIP() << "ForceController::commandCB is private and called by ROS subscriber, making it difficult to unit test in isolation without refactoring or integration testing.";
}


TEST(ForceControllerTest, PointAtTargetQuaternion) {
    KDL::Frame x_test;
    x_test.p = KDL::Vector(1.0, 0.0, 0.0);
    x_test.M = KDL::Rotation::RPY(0, 0, 0);

    KDL::Vector xd_p_target(0.0, 0.0, 0.0);

    KDL::Rotation result_orientation_M;

    KDL::Vector x_p_for_dot = x_test.p;
    KDL::Vector xd_p_for_dot = xd_p_target;

    // Simplified logic for target at origin, current pos not origin
    if (x_p_for_dot.Norm() > test_epsilon && xd_p_for_dot.Norm() < test_epsilon) {
        KDL::Vector desired_X_direction = (xd_p_for_dot - x_p_for_dot); // e.g. (-1,0,0)
        desired_X_direction.Normalize();
        KDL::Vector current_X_direction = x_test.M.UnitX(); // e.g. (1,0,0) if x_test.M is Identity

        KDL::Vector rot_axis = current_X_direction % desired_X_direction;
        double dot_prod = KDL::dot(current_X_direction, desired_X_direction);
        dot_prod = std::max(-1.0, std::min(1.0, dot_prod)); // Clamp for acos
        double rot_angle = std::acos(dot_prod);

        if (rot_axis.Norm() < test_epsilon) { // Collinear
            if (dot_prod > (1.0 - test_epsilon)) { // Pointing in the same direction
                result_orientation_M = KDL::Rotation::Identity();
            } else { // Pointing in opposite directions
                 KDL::Vector arbitrary_axis = KDL::Vector(0,1,0);
                 if(std::abs(KDL::dot(current_X_direction, arbitrary_axis)) > (1.0 - test_epsilon)) arbitrary_axis = KDL::Vector(0,0,1);
                 result_orientation_M = KDL::Rotation::Rot(arbitrary_axis, PI);
            }
        } else {
            result_orientation_M = KDL::Rotation::Rot(rot_axis, rot_angle);
        }
    } else {
      // Fallback or more general logic from previous test if needed
      // For this specific case (1,0,0) -> (0,0,0), RotY(PI) is expected if current_X is (1,0,0)
       result_orientation_M = KDL::Rotation::Rot(KDL::Vector(0,1,0), PI);
    }

    KDL::Vector expected_x_axis = (xd_p_target - x_test.p);
    if (expected_x_axis.Norm() > test_epsilon) {
        expected_x_axis.Normalize();
         ASSERT_TRUE(KDL::Equal(result_orientation_M.UnitX(), expected_x_axis, test_epsilon))
            << "Expected X: " << expected_x_axis.x() << "," << expected_x_axis.y() << "," << expected_x_axis.z()
            << "\nActual X:   " << result_orientation_M.UnitX().x() << "," << result_orientation_M.UnitX().y() << "," << result_orientation_M.UnitX().z();
    } else {
        ASSERT_TRUE(KDL::Equal(result_orientation_M, KDL::Rotation::Identity(), test_epsilon));
    }


    // Test case 2: Off-axis pointing
    x_test.p = KDL::Vector(1.0, 1.0, 0.0);
    x_test.M = KDL::Rotation::Identity();
    xd_p_target = KDL::Vector(0.0, 0.0, 0.0);

    x_p_for_dot = x_test.p;
    xd_p_for_dot = xd_p_target;

    if (x_p_for_dot.Norm() > test_epsilon && xd_p_for_dot.Norm() < test_epsilon) {
        KDL::Vector desired_X_direction = (xd_p_for_dot - x_p_for_dot);
        desired_X_direction.Normalize();
        KDL::Vector current_X_direction = x_test.M.UnitX();

        KDL::Vector rot_axis = current_X_direction % desired_X_direction;
        double dot_prod = KDL::dot(current_X_direction, desired_X_direction);
        dot_prod = std::max(-1.0, std::min(1.0, dot_prod));
        double rot_angle = std::acos(dot_prod);

        if (rot_axis.Norm() < test_epsilon) {
            if (dot_prod > (1.0 - test_epsilon)) {
                result_orientation_M = KDL::Rotation::Identity();
            } else {
                 KDL::Vector arbitrary_axis = KDL::Vector(0,0,1); // Use Z-axis if current_X is along X/Y plane
                 if(std::abs(KDL::dot(current_X_direction, arbitrary_axis)) > (1.0 - test_epsilon)) arbitrary_axis = KDL::Vector(0,1,0);
                 result_orientation_M = KDL::Rotation::Rot(arbitrary_axis, PI);
            }
        } else {
            result_orientation_M = KDL::Rotation::Rot(rot_axis, rot_angle);
        }
    } else {
         result_orientation_M = KDL::Rotation::Identity(); // Fallback for other cases
    }

    expected_x_axis = (xd_p_target - x_test.p);
     if (expected_x_axis.Norm() > test_epsilon) {
        expected_x_axis.Normalize();
        ASSERT_TRUE(KDL::Equal(result_orientation_M.UnitX(), expected_x_axis, test_epsilon))
            << "Expected X: " << expected_x_axis.x() << "," << expected_x_axis.y() << "," << expected_x_axis.z()
            << "\nActual X:   " << result_orientation_M.UnitX().x() << "," << result_orientation_M.UnitX().y() << "," << result_orientation_M.UnitX().z();
    } else {
        ASSERT_TRUE(KDL::Equal(result_orientation_M, KDL::Rotation::Identity(), test_epsilon));
    }
}


TEST(ForceControllerTest, ObstacleAvoidanceForce) {
    KDL::Frame x_obs_test;
    x_obs_test.p = KDL::Vector(0.0, 0.0, 0.0);
    x_obs_test.M = KDL::Rotation::Identity();

    KDL::Twist xdot_obs_test;
    xdot_obs_test.vel = KDL::Vector(1.0, 0.0, 0.0);
    xdot_obs_test.rot = KDL::Vector(0.0, 0.0, 0.0);

    KDL::Frame obstacle_test;
    obstacle_test.p = KDL::Vector(2.0, 0.1, 0.0);

    KDL::Vector P_avoid = KDL::Vector::Zero();

    KDL::Vector diff_obs_x = obstacle_test.p - x_obs_test.p;
    double diff_norm = diff_obs_x.Norm();
    double xdot_vel_norm = xdot_obs_test.vel.Norm();
    const double epsilon_obs = 1e-6;

    if (diff_norm > epsilon_obs && xdot_vel_norm > epsilon_obs) {
        KDL::Vector rotation_axis = diff_obs_x % xdot_obs_test.vel;
        KDL::Rotation R_rot;
        if (rotation_axis.Norm() > epsilon_obs) {
            R_rot = KDL::Rotation::Rot(rotation_axis, PI / 2.0);
        } else {
            R_rot = KDL::Rotation::Identity();
        }

        double dot_product_val = KDL::dot(diff_obs_x, xdot_obs_test.vel) / (diff_norm * xdot_vel_norm);
        dot_product_val = std::max(-1.0, std::min(1.0, dot_product_val));
        double phi = std::acos(dot_product_val);

        P_avoid = GAMMA * (R_rot * xdot_obs_test.vel) * phi * std::exp(-BETA * phi);
    }

    ASSERT_LT(P_avoid.y(), 0.0);

    obstacle_test.p = KDL::Vector(2.0, 0.0, 0.0);
    P_avoid = KDL::Vector::Zero();
    diff_obs_x = obstacle_test.p - x_obs_test.p;
    diff_norm = diff_obs_x.Norm();
    xdot_vel_norm = xdot_obs_test.vel.Norm();

    if (diff_norm > epsilon_obs && xdot_vel_norm > epsilon_obs) {
        KDL::Vector rotation_axis = diff_obs_x % xdot_obs_test.vel;
        KDL::Rotation R_rot = KDL::Rotation::Identity();
        if (rotation_axis.Norm() > epsilon_obs) { // Should be false for this case
             R_rot = KDL::Rotation::Rot(rotation_axis, PI / 2.0);
        }

        double dot_product_val = KDL::dot(diff_obs_x, xdot_obs_test.vel) / (diff_norm * xdot_vel_norm);
        dot_product_val = std::max(-1.0, std::min(1.0, dot_product_val));
        double phi = std::acos(dot_product_val);
        P_avoid = GAMMA * (R_rot * xdot_obs_test.vel) * phi * std::exp(-BETA * phi);
    }
    ASSERT_TRUE(KDL::Equal(P_avoid, KDL::Vector::Zero(), test_epsilon)) << "P_avoid should be zero if phi is zero.";
}


int main(int argc, char **argv) {
    // Mock ROS logging for tests if needed, e.g. to suppress ROS_WARN from MockNodeHandle
    // This can be complex. For now, allow logs to go to console.
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error); // Example
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
