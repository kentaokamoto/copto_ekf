#ifndef COPTO_EKF__EKF_COMPONENT_HPP_
#define COPTO_EKF__EKF_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COPTO_EKF_EKF_COMPONENT_EXPORT __attribute__((dllexport))
#define COPTO_EKF_EKF_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COPTO_EKF_EKF_COMPONENT_EXPORT __declspec(dllexport)
#define COPTO_EKF_EKF_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COPTO_EKF_EKF_COMPONENT_BUILDING_DLL
#define COPTO_EKF_EKF_COMPONENT_PUBLIC COPTO_EKF__EKF_COMPONENT_EXPORT
#else
#define COPTO_EKF_EKF_COMPONENT_PUBLIC COPTO_EKF__EKF_COMPONENT_IMPORT
#endif
#define COPTO_EKF__EKF_COMPONENT_PUBLIC_TYPE COPTO_EKF__EKF_COMPONENT_PUBLIC
#define COPTO_EKF_EKF_COMPONENT_LOCAL
#else
#define COPTO_EKF_EKF_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COPTO_EKF_EKF_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COPTO_EKF_EKF_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COPTO_EKF_EKF_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COPTO_EKF_EKF_COMPONENT_PUBLIC
#define COPTO_EKF_EKF_COMPONENT_LOCAL
#endif
#define COPTO_EKF_EKF_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "sensor_msgs/msg/imu.hpp"

namespace copto_ekf
{
class EKFComponent : public rclcpp::Node
{
public:
  COPTO_EKF_EKF_COMPONENT_PUBLIC
  explicit EKFComponent(const rclcpp::NodeOptions & options);

  double dt = 0.001;
  bool initialized = false;
  Eigen::MatrixXd P;
  Eigen::VectorXd x;
  Eigen::VectorXd X;
  Eigen::VectorXd y;
  Eigen::VectorXd u;
  Eigen::MatrixXd I;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd M;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  Eigen::VectorXd cov;

  rclcpp::Time imutimestamp;

private:
  void IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void modelfunc();
  void jacobi();
  bool init();
  void update();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr Twistpublisher_;
};
}  // namespace copto_ekf

#endif  // COPTO_EKF__EKF_COMPONENT_HPP_