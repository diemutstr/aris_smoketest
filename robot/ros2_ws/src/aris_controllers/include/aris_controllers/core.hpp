// The control law and the reference of aris_joint_impedance_controller, without ROS.
//
// Everything that decides a torque is here, so that it can be compiled and tested on a
// machine without ROS (test/core_test.cpp).  The ROS controller only reads the arm, calls
// Core::update once per control tick (1 kHz) and writes the torques.
//
//   tau = K (q_d - q) + D (qd_d - qd) + J^T f_ff + coriolis
//
// Gravity is added by the robot itself (libfranka), as in the franka example controllers.
// q_d, qd_d and f_ff come from a stream of samples (t, q, qd, f); between two samples the
// reference is the cubic that matches q and qd at both, which is how the planner defines a
// trajectory.  The force is interpolated linearly.
//
// Holding: the reference stands still at q_hold with zero velocity and zero force.  A hold is
// latched by the hold service, by a starved stream, or by a tracking error; it is cleared only
// by resume (or a new activation).  While latched, new streams are refused.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace aris_controllers {

using Vec7 = Eigen::Matrix<double, 7, 1>;
using Vec3 = Eigen::Vector3d;
using Mat37 = Eigen::Matrix<double, 3, 7>;

struct Sample {
  uint32_t stream{0};  // a stream id larger than the current one starts a new stream
  bool last{false};    // the final sample of its stream
  double t{0.0};       // s, on the stream's own clock
  Vec7 q{Vec7::Zero()};
  Vec7 qd{Vec7::Zero()};
  Vec3 f{Vec3::Zero()};  // N, base frame, the force the arm applies at the pen tip
};

// Single producer (the subscriber thread), single consumer (the control loop); no locks, no
// allocation after construction.
class SampleQueue {
 public:
  explicit SampleQueue(std::size_t capacity) : buf_(capacity + 1) {}

  bool push(const Sample& s) {
    const std::size_t h = head_.load(std::memory_order_relaxed);
    const std::size_t next = (h + 1) % buf_.size();
    if (next == tail_.load(std::memory_order_acquire)) return false;  // full
    buf_[h] = s;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(Sample& s) {
    const std::size_t t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;  // empty
    s = buf_[t];
    tail_.store((t + 1) % buf_.size(), std::memory_order_release);
    return true;
  }

 private:
  std::vector<Sample> buf_;
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

// The cubic between two samples (power basis, as in aris_robot/stream.py), force linear.
inline void interpolate(const Sample& a, const Sample& b, double t, Vec7& q, Vec7& qd, Vec3& f) {
  const double h = b.t - a.t;
  const double x = std::clamp(t - a.t, 0.0, h);
  const Vec7 c2 = (3.0 * (b.q - a.q) / h - 2.0 * a.qd - b.qd) / h;
  const Vec7 c3 = (2.0 * (a.q - b.q) / h + a.qd + b.qd) / (h * h);
  q = a.q + x * (a.qd + x * (c2 + x * c3));
  qd = a.qd + x * (2.0 * c2 + 3.0 * x * c3);
  f = a.f + (b.f - a.f) * (x / h);
}

// The samples of the current stream that are still needed, in a preallocated ring.
class Reference {
 public:
  explicit Reference(std::size_t capacity) : ring_(capacity) {}

  void clear() { start_ = count_ = 0; has_last_ = false; }
  std::size_t size() const { return count_; }
  bool has_last() const { return has_last_; }
  const Sample& at(std::size_t i) const { return ring_[(start_ + i) % ring_.size()]; }
  const Sample& back() const { return at(count_ - 1); }

  // false: full, or not later than the sample before it (the sample is dropped).
  bool push(const Sample& s) {
    if (count_ == ring_.size() || has_last_) return false;
    if (count_ > 0 && !(s.t > back().t)) return false;
    ring_[(start_ + count_) % ring_.size()] = s;
    ++count_;
    has_last_ = s.last;
    return true;
  }

  // The reference at stream time tau.  Returns false when tau is at or past the newest
  // sample: then q is that sample's, qd is zero and f is its force.
  bool eval(double tau, Vec7& q, Vec7& qd, Vec3& f) {
    while (count_ >= 2 && at(1).t <= tau) {
      start_ = (start_ + 1) % ring_.size();
      --count_;
    }
    if (count_ >= 2) {
      interpolate(at(0), at(1), tau, q, qd, f);
      return true;
    }
    q = back().q;
    qd.setZero();
    f = back().f;
    return false;
  }

 private:
  std::vector<Sample> ring_;
  std::size_t start_{0};
  std::size_t count_{0};
  bool has_last_{false};
};

struct Params {
  Vec7 k{(Vec7() << 300, 300, 250, 250, 40, 40, 15).finished()};      // Nm/rad
  Vec7 d{(Vec7() << 30, 30, 25, 20, 4, 4, 1.5).finished()};           // Nm s/rad
  Vec7 tau_max{(Vec7() << 75, 75, 75, 75, 11, 11, 11).finished()};    // Nm, FR3 rated 87/12
  double tau_rate{990.0};            // Nm/s per joint; libfranka refuses 1000 and more
  double f_max{5.0};                 // N, largest fed-forward force
  double starve_timeout{0.02};       // s past the newest sample before the stream is starved
  double max_error{0.05};            // rad, tracking error on any joint that latches a hold
  double start_tolerance{0.01};      // rad, a new stream must start this close to q_d
  double idle_force_timeout{2.0};    // s idle after a stream before the force is taken away
  double idle_force_ramp{1.0};       // s to take f_max away
};

enum class Hold : uint8_t { kNone = 0, kRequested = 1, kStarved = 2, kTracking = 3 };

inline const char* hold_text(Hold h) {
  switch (h) {
    case Hold::kNone: return "";
    case Hold::kRequested: return "hold requested";
    case Hold::kStarved: return "starved: the reference stream ran dry";
    case Hold::kTracking: return "tracking error over the limit";
  }
  return "";
}

struct Status {
  uint32_t stream{0};      // the stream being or last followed
  uint32_t rejected{0};    // the last stream refused (latched hold, or not starting at q_d)
  double t{0.0};           // its clock
  bool streaming{false};
  bool done{false};        // the stream reached its final sample and holds there
  bool starved{false};
  Hold hold{Hold::kNone};
  int error_joint{-1};     // 0-based joint that tripped the tracking limit
  Vec7 q_d{Vec7::Zero()};
  Vec7 qd_d{Vec7::Zero()};
  Vec7 error{Vec7::Zero()};
  Vec3 f_ff{Vec3::Zero()};
  Vec7 tau{Vec7::Zero()};
};

class Core {
 public:
  Core(const Params& p, std::size_t capacity) : p_(p), ref_(capacity) {}

  const Status& status() const { return s_; }
  const Params& params() const { return p_; }

  // Stand still at q; forget every stream and every hold.
  void activate(const Vec7& q) {
    ref_.clear();
    s_ = Status();
    s_.q_d = q;
    idle_ = 0.0;
    last_tau_.setZero();
  }

  void request_hold(const Vec7& q) { latch(Hold::kRequested, q); }

  void resume(const Vec7& q) {
    s_.hold = Hold::kNone;
    s_.starved = false;
    s_.error_joint = -1;
    s_.q_d = q;
    s_.qd_d.setZero();
    s_.f_ff.setZero();
  }

  // One sample from the queue.  A sample of a newer stream starts that stream, if the arm
  // may move and the stream starts at the current reference.
  void offer(const Sample& x) {
    if (x.stream == s_.rejected && x.stream != 0) return;
    if (x.stream > s_.stream) {
      if (s_.hold != Hold::kNone ||
          (x.q - s_.q_d).cwiseAbs().maxCoeff() > p_.start_tolerance) {
        s_.rejected = x.stream;
        return;
      }
      ref_.clear();
      s_.stream = x.stream;
      s_.t = x.t;
      s_.streaming = true;
      s_.done = false;
      fresh_ = true;
      idle_ = 0.0;
    }
    if (x.stream == s_.stream && s_.streaming) ref_.push(x);
  }

  // One control tick.  q, qd measured; J the 3x7 Jacobian of the pen tip's position (base
  // frame); coriolis from the model; dt the tick.  Returns the torques to command.
  Vec7 update(const Vec7& q, const Vec7& qd, const Mat37& J, const Vec7& coriolis, double dt) {
    if (s_.hold == Hold::kNone) follow(dt);
    if (s_.hold != Hold::kNone) {
      s_.qd_d.setZero();
      s_.f_ff.setZero();
    }
    s_.error = s_.q_d - q;
    Eigen::Index j = 0;
    if (s_.hold == Hold::kNone && s_.error.cwiseAbs().maxCoeff(&j) > p_.max_error) {
      s_.error_joint = static_cast<int>(j);
      latch(Hold::kTracking, q);
      s_.error.setZero();
    }
    Vec3 f = s_.f_ff;
    const double n = f.norm();
    if (n > p_.f_max) f *= p_.f_max / n;
    Vec7 tau = p_.k.cwiseProduct(s_.error) + p_.d.cwiseProduct(s_.qd_d - qd) +
               J.transpose() * f + coriolis;
    s_.tau = saturate(tau, dt);
    return s_.tau;
  }

 private:
  void follow(double dt) {
    if (!s_.streaming) {
      idle_ += dt;
      if (idle_ > p_.idle_force_timeout) {  // nobody is streaming: take the force away
        const double step = p_.f_max * dt / p_.idle_force_ramp;
        const double n = s_.f_ff.norm();
        s_.f_ff = n <= step ? Vec3::Zero() : Vec3(s_.f_ff * (1.0 - step / n));
      }
      return;
    }
    if (!fresh_) s_.t += dt;  // the first tick of a stream reads its first sample
    fresh_ = false;
    if (ref_.eval(s_.t, s_.q_d, s_.qd_d, s_.f_ff)) return;
    if (ref_.has_last()) {
      s_.streaming = false;
      s_.done = true;
    } else if (s_.t - ref_.back().t > p_.starve_timeout) {
      s_.starved = true;
      latch(Hold::kStarved, s_.q_d);
    }
  }

  void latch(Hold why, const Vec7& q) {
    s_.hold = why;
    s_.streaming = false;
    s_.q_d = q;
    s_.qd_d.setZero();
    s_.f_ff.setZero();
    ref_.clear();
  }

  // Per-joint magnitude, then rate: libfranka faults on a torque step over its rate limit.
  Vec7 saturate(const Vec7& tau, double dt) {
    const double step = p_.tau_rate * dt;
    for (int i = 0; i < 7; ++i) {
      const double lim = std::clamp(tau(i), -p_.tau_max(i), p_.tau_max(i));
      last_tau_(i) += std::clamp(lim - last_tau_(i), -step, step);
    }
    return last_tau_;
  }

  Params p_;
  Reference ref_;
  Status s_;
  double idle_{0.0};
  bool fresh_{false};
  Vec7 last_tau_{Vec7::Zero()};
};

}  // namespace aris_controllers
