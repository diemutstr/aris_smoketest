#include "aris_controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace aris_controllers {

namespace {

constexpr int kJoints = 7;

// franka_hardware hands its robot state out as a pointer stored in a double (as the
// franka_semantic_components do it).
const franka::RobotState* state_pointer(double value) {
  const franka::RobotState* p = nullptr;
  static_assert(sizeof(p) == sizeof(value), "pointer and double must have the same size");
  std::memcpy(&p, &value, sizeof(p));
  return p;
}

Vec7 vec7(const std::vector<double>& v) {
  Vec7 out;
  for (int i = 0; i < kJoints; ++i) out(i) = v.at(i);
  return out;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& r) {
  Eigen::Matrix3d m;
  m << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
  return m;
}

}  // namespace

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kJoints; ++i) config.names.push_back(joint(i) + "/effort");
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kJoints; ++i) config.names.push_back(joint(i) + "/position");
  for (int i = 1; i <= kJoints; ++i) config.names.push_back(joint(i) + "/velocity");
  if (use_model_ && model_) {
    for (const auto& name : model_->get_state_interface_names()) config.names.push_back(name);
  }
  return config;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    const Params d;
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", std::vector<double>(d.k.data(), d.k.data() + 7));
    auto_declare<std::vector<double>>("d_gains", std::vector<double>(d.d.data(), d.d.data() + 7));
    auto_declare<std::vector<double>>("max_torques",
                                      std::vector<double>(d.tau_max.data(), d.tau_max.data() + 7));
    auto_declare<double>("max_torque_rate", d.tau_rate);
    auto_declare<double>("max_force", d.f_max);
    auto_declare<double>("starve_timeout", d.starve_timeout);
    auto_declare<double>("max_tracking_error", d.max_error);
    auto_declare<double>("start_tolerance", d.start_tolerance);
    auto_declare<double>("idle_force_timeout", d.idle_force_timeout);
    auto_declare<double>("idle_force_ramp", d.idle_force_ramp);
    auto_declare<double>("k_normal", d.k_normal);
    auto_declare<double>("d_normal", d.d_normal);
    auto_declare<double>("mass_normal", d.mass_normal);
    auto_declare<std::vector<double>>("tip_offset_flange", {0.0, 0.0, 0.0});
    auto_declare<bool>("use_model", true);
    auto_declare<double>("status_rate", 250.0);
    auto_declare<int>("queue_capacity", 8192);
  } catch (const std::exception& e) {
    fprintf(stderr, "aris_joint_impedance_controller: on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();
  auto logger = node->get_logger();
  Params p;
  try {
    arm_id_ = node->get_parameter("arm_id").as_string();
    p.k = vec7(node->get_parameter("k_gains").as_double_array());
    p.d = vec7(node->get_parameter("d_gains").as_double_array());
    p.tau_max = vec7(node->get_parameter("max_torques").as_double_array());
    const auto tip = node->get_parameter("tip_offset_flange").as_double_array();
    tip_flange_ = Eigen::Vector3d(tip.at(0), tip.at(1), tip.at(2));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "gains need 7 numbers each and tip_offset_flange 3 (%s)", e.what());
    return CallbackReturn::FAILURE;
  }
  p.tau_rate = node->get_parameter("max_torque_rate").as_double();
  p.f_max = node->get_parameter("max_force").as_double();
  p.starve_timeout = node->get_parameter("starve_timeout").as_double();
  p.max_error = node->get_parameter("max_tracking_error").as_double();
  p.start_tolerance = node->get_parameter("start_tolerance").as_double();
  p.idle_force_timeout = node->get_parameter("idle_force_timeout").as_double();
  p.idle_force_ramp = node->get_parameter("idle_force_ramp").as_double();
  p.k_normal = node->get_parameter("k_normal").as_double();
  p.d_normal = node->get_parameter("d_normal").as_double();
  p.mass_normal = node->get_parameter("mass_normal").as_double();
  use_model_ = node->get_parameter("use_model").as_bool();
  status_period_ = 1.0 / std::max(1.0, node->get_parameter("status_rate").as_double());
  const int capacity = std::max(1024, static_cast<int>(node->get_parameter("queue_capacity").as_int()));
  if ((p.k.array() <= 0).any() || (p.d.array() <= 0).any() || p.tau_rate <= 0 ||
      p.tau_rate >= 1000.0 || p.f_max < 0 || p.max_error <= 0 || p.k_normal < 0 ||
      p.mass_normal <= 0) {
    RCLCPP_FATAL(logger, "gains must be > 0, 0 < max_torque_rate < 1000, max_tracking_error > 0, "
                         "k_normal >= 0, mass_normal > 0");
    return CallbackReturn::FAILURE;
  }

  core_ = std::make_unique<Core>(p, static_cast<std::size_t>(capacity));
  queue_ = std::make_unique<SampleQueue>(static_cast<std::size_t>(capacity));
  if (use_model_) {
    model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_ + "/robot_state");
  } else {
    model_.reset();
    RCLCPP_WARN(logger, "use_model is false: no coriolis, no pen force, no force estimate "
                        "(for fake hardware only)");
  }

  reference_sub_ = node->create_subscription<aris_msgs::msg::Reference>(
      "~/reference", rclcpp::QoS(100).reliable(),
      [this](const aris_msgs::msg::Reference::SharedPtr msg) { on_reference(*msg); });
  hold_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/hold", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        hold_request_ = true;
        res->success = true;
        res->message = "hold requested";
      });
  resume_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/resume", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        resume_request_ = true;
        res->success = true;
        res->message = "resume requested";
      });
  status_pub_ = node->create_publisher<StatusMsg>("~/status", rclcpp::SystemDefaultsQoS());
  status_rt_ = std::make_unique<realtime_tools::RealtimePublisher<StatusMsg>>(status_pub_);

  RCLCPP_INFO(logger, "configured: arm_id=%s, K=[%.0f %.0f %.0f %.0f %.0f %.0f %.0f], "
              "tip in flange [%.4f %.4f %.4f], max force %.1f N",
              arm_id_.c_str(), p.k(0), p.k(1), p.k(2), p.k(3), p.k(4), p.k(5), p.k(6),
              tip_flange_.x(), tip_flange_.y(), tip_flange_.z(), p.f_max);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
  robot_state_ = nullptr;
  if (use_model_) {
    model_->assign_loaned_state_interfaces(state_interfaces_);
    for (const auto& si : state_interfaces_) {
      if (si.get_name() == arm_id_ + "/robot_state") {
        robot_state_ = state_pointer(si.get_optional().value());
      }
    }
    if (robot_state_ == nullptr) {
      RCLCPP_FATAL(get_node()->get_logger(), "no %s/robot_state interface", arm_id_.c_str());
      return CallbackReturn::FAILURE;
    }
  }
  Vec7 q, qd;
  read_joints(q, qd);
  // the start tolerance may have been set for this job since configure (the driver does it
  // before switching the controller in)
  core_->set_start_tolerance(get_node()->get_parameter("start_tolerance").as_double());
  core_->activate(q);                 // hold where the arm stands; no jump
  Sample stale;
  while (queue_->pop(stale)) {
  }
  hold_request_ = resume_request_ = false;
  since_status_ = status_period_;
  RCLCPP_INFO(get_node()->get_logger(), "active, holding where the arm stands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
  if (model_) model_->release_interfaces();
  robot_state_ = nullptr;
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::read_joints(Vec7& q, Vec7& qd) const {
  for (int i = 0; i < kJoints; ++i) {
    q(i) = state_interfaces_[i].get_optional().value();
    qd(i) = state_interfaces_[kJoints + i].get_optional().value();
  }
}

void JointImpedanceController::read_model(Mat37& J, Vec7& coriolis, Vec3& force, Mat7& M) const {
  J.setZero();
  coriolis.setZero();
  force.setZero();
  M.setZero();
  if (!use_model_ || robot_state_ == nullptr) return;
  const std::array<double, 49> m = model_->getMassMatrix();   // column-major
  M = Eigen::Map<const Mat7>(m.data());
  const std::array<double, 7> c = model_->getCoriolisForceVector();
  const std::array<double, 42> j = model_->getZeroJacobian(franka::Frame::kFlange);
  const std::array<double, 16> t = model_->getPoseMatrix(franka::Frame::kFlange);
  coriolis = Eigen::Map<const Vec7>(c.data());
  const Eigen::Map<const Eigen::Matrix<double, 6, 7>> J6(j.data());
  const Eigen::Map<const Eigen::Matrix4d> T(t.data());
  // the pen tip moves with the flange: v_tip = v_flange + w x r, r the tip from the flange
  const Eigen::Vector3d r = T.topLeftCorner<3, 3>() * tip_flange_;
  J = J6.topRows<3>() - skew(r) * J6.bottomRows<3>();
  for (int i = 0; i < 3; ++i) force(i) = robot_state_->O_F_ext_hat_K[i];
}

void JointImpedanceController::on_reference(const aris_msgs::msg::Reference& msg) {
  const std::size_t n = msg.t.size();
  const bool normals = msg.n.size() == 3 * n;
  if (n == 0 || msg.q.size() != 7 * n || msg.qd.size() != 7 * n || msg.f.size() != 3 * n ||
      (!msg.n.empty() && !normals)) {
    RCLCPP_WARN(get_node()->get_logger(), "reference chunk with inconsistent sizes ignored");
    return;
  }
  Sample s;
  s.stream = msg.stream;
  for (std::size_t k = 0; k < n; ++k) {
    s.t = msg.t[k];
    s.q = Eigen::Map<const Vec7>(&msg.q[7 * k]);
    s.qd = Eigen::Map<const Vec7>(&msg.qd[7 * k]);
    s.f = Eigen::Map<const Vec3>(&msg.f[3 * k]);
    s.n = normals ? Vec3(Eigen::Map<const Vec3>(&msg.n[3 * k])) : Vec3::Zero();
    s.last = msg.last && k + 1 == n;
    if (!queue_->push(s)) {
      ++dropped_;  // the controller will starve and hold; the driver sees it in the status
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "reference queue full, samples dropped");
      return;
    }
  }
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double dt = std::clamp(period.seconds(), 1e-4, 1e-2);
  Vec7 q, qd;
  read_joints(q, qd);
  if (hold_request_.exchange(false)) core_->request_hold(q);
  if (resume_request_.exchange(false)) core_->resume(q);
  Sample x;
  while (queue_->pop(x)) core_->offer(x);

  Mat37 J;
  Vec7 coriolis;
  Vec3 force;
  Mat7 M;
  read_model(J, coriolis, force, M);
  const Vec7 tau = core_->update(q, qd, J, coriolis, dt, M);
  for (int i = 0; i < kJoints; ++i) {
    (void)command_interfaces_[i].set_value(tau(i));
  }

  since_status_ += dt;
  if (since_status_ >= status_period_) {
    since_status_ = 0.0;
    publish_status(time, force);
  }
  return controller_interface::return_type::OK;
}

void JointImpedanceController::publish_status(const rclcpp::Time& now, const Vec3& force) {
  if (!status_rt_->trylock()) return;
  const Status& s = core_->status();
  StatusMsg& m = status_rt_->msg_;
  m.stamp = now;
  m.stream = s.stream;
  m.rejected = s.rejected;
  m.t = s.t;
  m.streaming = s.streaming;
  m.done = s.done;
  m.starved = s.starved;
  m.holding = s.hold != Hold::kNone;
  m.reason = hold_text(s.hold);  // short literals; the string keeps its capacity
  m.error_joint = s.error_joint;
  m.pen_down = s.normal_spring;
  m.d_normal = s.d_normal;
  for (int i = 0; i < kJoints; ++i) {
    m.q_d[i] = s.q_d(i);
    m.qd_d[i] = s.qd_d(i);
    m.tracking_error[i] = s.error(i);
    m.tau[i] = s.tau(i);
  }
  for (int i = 0; i < 3; ++i) {
    m.f_ff[i] = s.f_ff(i);
    m.force[i] = force(i);
  }
  status_rt_->unlockAndPublish();
}

}  // namespace aris_controllers

PLUGINLIB_EXPORT_CLASS(aris_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
