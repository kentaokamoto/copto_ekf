#include <Eigen/Dense>
#include <copto_ekf/ekf_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace copto_ekf
{
EKFComponent::EKFComponent(const rclcpp::NodeOptions & options) : Node("copto_ekf_node", options)
{
  
  A = Eigen::MatrixXd::Zero(10, 10);
  B = Eigen::MatrixXd::Zero(10, 6);
  C = Eigen::MatrixXd::Zero(10, 10);
  M = Eigen::MatrixXd::Zero(6, 6);
  Q = Eigen::MatrixXd::Zero(10, 10);
  K = Eigen::MatrixXd::Zero(10, 10);
  S = Eigen::MatrixXd::Zero(10, 10);
  P = Eigen::MatrixXd::Zero(10, 10);
  I = Eigen::MatrixXd::Identity(10, 10);
  X = Eigen::VectorXd::Zero(10);
  x = Eigen::VectorXd::Zero(10);
  y = Eigen::VectorXd::Zero(10);
  u = Eigen::VectorXd::Zero(6);
  cov = Eigen::VectorXd::Zero(36);

  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

    Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 1);

    Twistpublisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/twist", 1);
}


void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imutimestamp = msg->header.stamp;
  u(0) = msg->linear_acceleration.x;
  u(1) = msg->linear_acceleration.y;
  u(2) = msg->linear_acceleration.z;
  u(3) = msg->angular_velocity.x;
  u(4) = msg->angular_velocity.y;
  u(5) = msg->angular_velocity.z;
    if(!initialized){init();}
    update();
}

bool EKFComponent::init()
{
  x << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  P << 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 100;
  return initialized = true;
}

void EKFComponent::modelfunc()
{
  double xx, xy, xz, vx, vy, vz, q0, q1, q2, q3;
  xx = x(0);
  xy = x(1);
  xz = x(2);
  vx = x(3);
  vy = x(4);
  vz = x(5);
  q0 = x(6);
  q1 = x(7);
  q2 = x(8);
  q3 = x(9);

  x(0) = xx + vx * dt;
  x(1) = xy + vy * dt;
  x(2) = xz + vz * dt;

  x(3) = vx + ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * u(0) + (2 * q1 * q2 - 2 * q0 * q3) * u(1) +
               (2 * q1 * q3 - 2 * q0 * q2) * u(2)) *
                dt;
  x(4) = vy + ((2 * q1 * q2 + 2 * q0 * q3) * u(0) + (q0 * q0 + q2 * q2 - q1 * q1 - q3 * q3) * u(1) +
               (2 * q2 * q3 - 2 * q0 * q1) * u(2)) *
                dt;
  x(5) = vz + ((2 * q1 * q3 - 2 * q0 * q2) * u(0) + (2 * q2 * q3 + 2 * q0 * q1) * u(1) +
               (q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * u(2) - 9.797) *
                dt;

  x(6) = (-u(3) * q1 - u(4) * q2 - u(5) * q3) * dt + q0;
  x(7) = (u(3) * q0 + u(5) * q2 - u(4) * q3) * dt + q1;
  x(8) = (u(4) * q0 - u(5) * q1 + u(3) * q2) * dt + q2;
  x(9) = (u(5) * q0 + u(4) * q1 - u(3) * q2) * dt + q3;
}

void EKFComponent::jacobi()
{
  A << 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, (2 * x(6) * u(0) - 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt,
    (-2 * x(8) * u(0) + 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt,
    (-2 * x(9) * u(0) - 2 * x(6) * u(1) + 2 * x(7) * u(2)) * dt, 0, 0, 0, 0, 1, 0,
    (2 * x(9) * u(0) + 2 * x(6) * u(1) - 2 * x(7) * u(2)) * dt,
    (2 * x(8) * u(0) - 2 * x(7) * u(1) - 2 * x(6) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt,
    (2 * x(6) * u(0) - 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt, 0, 0, 0, 0, 0, 1,
    (-2 * x(9) * u(0) + 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt,
    (2 * x(9) * u(0) + 2 * x(6) * u(1) - 2 * x(7) * u(2)) * dt,
    (-2 * x(6) * u(0) + 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt, 0, 0, 0, 0, 0, 0, 1, -dt * u(3),
    -dt * u(4), -dt * u(5), 0, 0, 0, 0, 0, 0, dt * u(3), 1, dt * u(5), -dt * u(4), 0, 0, 0, 0, 0, 0,
    dt * u(4), -dt * u(5), 1, dt * u(3), 0, 0, 0, 0, 0, 0, dt * u(5), dt * u(4), -dt * u(3), 1;

  B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)) * dt,
    2 * (x(7) * x(8) - x(6) * x(9)) * dt, 2 * (x(7) * x(9) + x(6) * x(8)) * dt, 0, 0, 0,
    2 * (x(7) * x(8) - x(6) * x(9)) * dt,
    (x(6) * x(6) - x(7) * x(7) + x(8) * x(8) - x(9) * x(9)) * dt,
    2 * (x(8) * x(9) + x(6) * x(7)) * dt, 0, 0, 0, 2 * (x(7) * x(9) + x(6) * x(8)) * dt,
    2 * (x(8) * x(9) - x(6) * x(7)) * dt,
    (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)) * dt, 0, 0, 0, 0, 0, 0, -x(7) * dt,
    x(8) * dt, -x(9) * dt, 0, 0, 0, x(6) * dt, -x(9) * dt, x(8) * dt, 0, 0, 0, x(9) * dt, x(6) * dt,
    -x(7) * dt, 0, 0, 0, -x(8) * dt, x(7) * dt, x(6) * dt;

  C <<  1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  M << 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0,
    0, 0, 0, 0, 100;

  // if you have GPS covariance, you can use here.
  /*
  Q << cov(0), cov(1), cov(2), 0, 0, 0, 0, cov(3), cov(4), cov(5), cov(6), cov(7), cov(8), 0, 0, 0,
    0, cov(9), cov(10), cov(11), cov(12), cov(13), cov(14), 0, 0, 0, 0, cov(15), cov(16), cov(17),
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, cov(18), cov(19), cov(20), 0, 0, 0, 0, cov(21), cov(22), cov(23),
    cov(24), cov(25), cov(26), 0, 0, 0, 0, cov(27), cov(28), cov(29), cov(30), cov(31), cov(32), 0,
    0, 0, 0, cov(33), cov(34), cov(35);
  */

  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1;
}

void EKFComponent::update()
{
    if (!initialized) {
        std::cout << "NOT Initialized" << std::endl;
    }

    // 予測ステップ
    X = A * X + x - A * x;
    y = C * X;

    modelfunc();
    jacobi();
    
    // filtering step 1
    P = A * P * A.transpose() + B * M * B.transpose();
    S = C * P * C.transpose() + Q;
    K = P * C.transpose() * S.inverse();
    x = x + K * (y - C * x);
    P = (I - K * C) * P;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "/map";
    pose_msg.header.stamp = imutimestamp;
    pose_msg.pose.pose.position.x = x(0);
    pose_msg.pose.pose.position.y = x(1);
    pose_msg.pose.pose.position.z = x(2);

    pose_msg.pose.pose.orientation.w = x(6);
    pose_msg.pose.pose.orientation.x = x(7);
    pose_msg.pose.pose.orientation.y = x(8);
    pose_msg.pose.pose.orientation.z = x(9);

    pose_msg.pose.covariance = {
        P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
        P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
        P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
        P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};

    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.frame_id = "/map";
    twist_msg.header.stamp = imutimestamp;
    twist_msg.twist.twist.linear.x = x(3);
    twist_msg.twist.twist.linear.y = x(4);
    twist_msg.twist.twist.linear.z = x(5);
    twist_msg.twist.covariance = {
        P(3, 3), P(3, 4), P(3, 5), 0, 0, 0, P(4, 3), P(4, 4), P(4, 5),
        0, 0, 0, P(5, 3), P(5, 4), P(5, 5), 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    Posepublisher_->publish(pose_msg);
    Twistpublisher_->publish(twist_msg);
}
}  // namespace copto_ekf

RCLCPP_COMPONENTS_REGISTER_NODE(copto_ekf::EKFComponent)