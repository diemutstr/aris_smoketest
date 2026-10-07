// The control law and the reference of aris_joint_impedance_controller, without ROS.
//
// Everything that decides a torque is here, so that it can be compiled and tested on a
// machine without ROS (test/core_test.cpp).  The ROS controller only reads the arm, calls
// Core::update once per control tick (1 kHz) and writes the torques.
//
//   tau = K e + D ed - J^T (B_k J e + B_d J ed) + J^T f_ff + coriolis
//   e = q_d - q, ed = qd_d - qd
//
// Pen down (the sample carries a paper normal n): the tip stiffness the joint springs give,
// K_t = (J K^-1 J^T)^-1, is replaced along n by a soft spring k_n, and likewise the damping.
// B_k = K_t - P K_t P - k_n n n^T with P = I - n n^T, so that the tip stiffness becomes exactly
//   (J (K - J^T B_k J)^-1 J^T)^-1 = K_t - B_k = P K_t P + k_n n n^T      (push-through identity)
// i.e. k_n along the normal, the paper-plane block of K_t unchanged, and no coupling between
// the normal and the plane.  It is symmetric, and the joint stiffness K - J^T B_k J stays
// positive definite (e^T K e >= (Je)^T K_t (Je)), so the law stays passive.  Cost per tick:
// two 3x3 inverses.  J is the 3x7 Jacobian of the pen tip's position.  Pen up (n = 0): plain
// joint impedance.
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
  Vec3 n{Vec3::Zero()};  // the paper normal (base frame, unit) while the pen is down, else 0
};

using Mat3 = Eigen::Matrix3d;
using Mat7 = Eigen::Matrix<double, 7, 7>;

// The tip-space term that replaces the stiffness (or damping) T along the unit normal n by
// k: T - P T P - k n n^T.
inline Mat3 normal_replacement(const Mat3& T, const Vec3& n, double k) {
  const Mat3 P = Mat3::Identity() - n * n.transpose();
  return T - P * T * P - k * n * n.transpose();
}

// (J W^-1 J^T)^-1 for a diagonal W > 0: the tip-space image of joint springs W.  false when
// J has lost rank (then the normal spring is not applied).
inline bool tip_space(const Mat37& J, const Vec7& w, Mat3& out) {
  const Mat3 C = J * w.cwiseInverse().asDiagonal() * J.transpose();
  Eigen::LDLT<Mat3> ldlt(C);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() || C.determinant() < 1e-18) return false;
  out = ldlt.solve(Mat3::Identity());
  return true;
}

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

// The cubic between two samples (power basis, as in aris_robot/stream.py), force linear,
// the normal of the earlier sample.
inline void interpolate(const Sample& a, const Sample& b, double t, Vec7& q, Vec7& qd, Vec3& f,
                        Vec3& n) {
  const double h = b.t - a.t;
  const double x = std::clamp(t - a.t, 0.0, h);
  const Vec7 c2 = (3.0 * (b.q - a.q) / h - 2.0 * a.qd - b.qd) / h;
  const Vec7 c3 = (2.0 * (a.q - b.q) / h + a.qd + b.qd) / (h * h);
  q = a.q + x * (a.qd + x * (c2 + x * c3));
  qd = a.qd + x * (2.0 * c2 + 3.0 * x * c3);
  f = a.f + (b.f - a.f) * (x / h);
  n = a.n;
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
  bool eval(double tau, Vec7& q, Vec7& qd, Vec3& f, Vec3& n) {
    while (count_ >= 2 && at(1).t <= tau) {
      start_ = (start_ + 1) % ring_.size();
      --count_;
    }
    if (count_ >= 2) {
      interpolate(at(0), at(1), tau, q, qd, f, n);
      return true;
    }
    q = back().q;
    qd.setZero();
    f = back().f;
    n = back().n;
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
  double k_normal{100.0};            // N/m, the pen's spring along the paper normal
  double d_normal{-1.0};             // N s/m along the normal; < 0: critically damped
  double mass_normal{3.0};           // kg along the normal when no mass matrix is given
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
  bool normal_spring{false};  // the pen-down law was applied this tick
  Hold hold{Hold::kNone};
  int error_joint{-1};     // 0-based joint that tripped the tracking limit
  Vec7 q_d{Vec7::Zero()};
  Vec7 qd_d{Vec7::Zero()};
  Vec7 error{Vec7::Zero()};
  Vec3 f_ff{Vec3::Zero()};
  Vec3 n{Vec3::Zero()};    // the paper normal of the reference (0: pen up)
  double d_normal{0.0};    // the damping along the normal used this tick
  Vec7 tau{Vec7::Zero()};
};

class Core {
 public:
  Core(const Params& p, std::size_t capacity) : p_(p), ref_(capacity) {}

  const Status& status() const { return s_; }
  const Params& params() const { return p_; }

  // The job's start tolerance (rig.json / the job header), set by the driver before the
  // controller is switched in.
  void set_start_tolerance(double rad) { p_.start_tolerance = rad; }

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
    s_.n.setZero();
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
  // frame); coriolis from the model; M the mass matrix (zero if unknown); dt the tick.
  // Returns the torques to command.
  Vec7 update(const Vec7& q, const Vec7& qd, const Mat37& J, const Vec7& coriolis, double dt,
              const Mat7& M = Mat7::Zero()) {
    if (s_.hold == Hold::kNone) follow(dt);
    if (s_.hold != Hold::kNone) {
      s_.qd_d.setZero();
      s_.f_ff.setZero();
      s_.n.setZero();
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
    const Vec7 ed = s_.qd_d - qd;
    Vec7 tau = p_.k.cwiseProduct(s_.error) + p_.d.cwiseProduct(ed) + J.transpose() * f +
               coriolis - normal_term(J, s_.error, ed, M);
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
    if (ref_.eval(s_.t, s_.q_d, s_.qd_d, s_.f_ff, s_.n)) return;
    if (ref_.has_last()) {
      s_.streaming = false;
      s_.done = true;
    } else if (s_.t - ref_.back().t > p_.starve_timeout) {
      s_.starved = true;
      latch(Hold::kStarved, s_.q_d);
    }
  }

  // J^T (B_k J e + B_d J ed), see the top of the file; zero with the pen up.
  Vec7 normal_term(const Mat37& J, const Vec7& e, const Vec7& ed, const Mat7& M) {
    s_.normal_spring = false;
    const double nn = s_.n.norm();
    Mat3 Kt, Dt;
    if (nn < 0.5 || !tip_space(J, p_.k, Kt) || !tip_space(J, p_.d, Dt)) return Vec7::Zero();
    const Vec3 n = s_.n / nn;
    double m = p_.mass_normal;
    if (M.squaredNorm() > 0.0) {  // the arm's own mass along the normal, at the tip
      const double inv = n.dot(J * M.ldlt().solve(J.transpose() * n));
      if (inv > 1e-9) m = 1.0 / inv;
    }
    s_.d_normal = p_.d_normal >= 0.0 ? p_.d_normal : 2.0 * std::sqrt(p_.k_normal * m);
    s_.normal_spring = true;
    return J.transpose() * (normal_replacement(Kt, n, p_.k_normal) * (J * e) +
                            normal_replacement(Dt, n, s_.d_normal) * (J * ed));
  }

  void latch(Hold why, const Vec7& q) {
    s_.hold = why;
    s_.streaming = false;
    s_.q_d = q;
    s_.qd_d.setZero();
    s_.f_ff.setZero();
    s_.n.setZero();
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
