// aris_joint_impedance_controller: joint impedance around a streamed joint trajectory, with the
// pen force fed forward at the tip.  One FR3 through franka_hardware (ROS 2 Jazzy).
//
//   tau = K (q_d - q) + D (qd_d - qd) + J_tip^T f_ff + coriolis     (gravity: by the robot)
// with, while the pen is down, the tip's stiffness and damping along the paper normal replaced
// by a soft spring k_normal (see core.hpp).
//
// The law, the reference and the holds are in core.hpp (ROS-free, tested without ROS).  This
// class only wires them to ros2_control:
//   ~/reference  aris_msgs/Reference        chunks of samples (t, q_d, qd_d, f_ff)
//   ~/status     aris_msgs/ImpedanceStatus  at `status_rate`
//   ~/hold       std_srvs/Trigger           latch a hold where the arm is
//   ~/resume     std_srvs/Trigger           clear a hold; stand still where the arm is
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aris_controllers/core.hpp"
#include "aris_msgs/msg/impedance_status.hpp"
#include "aris_msgs/msg/reference.hpp"
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_robot_model.hpp"

namespace aris_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JointImpedanceController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  using StatusMsg = aris_msgs::msg::ImpedanceStatus;

  std::string joint(int i) const { return arm_id_ + "_joint" + std::to_string(i); }
  void read_joints(Vec7& q, Vec7& qd) const;
  void read_model(Mat37& J, Vec7& coriolis, Vec3& force, Mat7& M) const;
  void on_reference(const aris_msgs::msg::Reference& msg);
  void publish_status(const rclcpp::Time& now, const Vec3& force);

  std::string arm_id_{"fr3"};
  bool use_model_{true};
  Eigen::Vector3d tip_flange_{Eigen::Vector3d::Zero()};  // pen tip in the flange frame
  double status_period_{0.004};
  double since_status_{0.0};

  std::unique_ptr<Core> core_;
  std::unique_ptr<SampleQueue> queue_;
  std::atomic<bool> hold_request_{false};
  std::atomic<bool> resume_request_{false};
  std::atomic<uint64_t> dropped_{0};

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> model_;
  const franka::RobotState* robot_state_{nullptr};

  rclcpp::Subscription<aris_msgs::msg::Reference>::SharedPtr reference_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  std::shared_ptr<rclcpp::Publisher<StatusMsg>> status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<StatusMsg>> status_rt_;
};

}  // namespace aris_controllers
