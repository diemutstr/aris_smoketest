// Tests of the controller core without ROS.  Build and run (robot/tests/test_controller_core.py
// does this):
//   g++ -std=c++17 -O2 -I<eigen3> -I../include core_test.cpp -o core_test
//   ./core_test                      scenario checks; prints "PASS n" or the first failure
//   ./core_test probe < poses.txt
//       reads "J(3x7 row-major) n(3)" lines; prints per line the joint stiffness and damping
//       matrices the law applies with the pen down (49 + 49 numbers, row-major) and d_normal
//   ./core_test follow <dt> < samples.txt
//       reads "t q1..q7 qd1..qd7 f1..f3" lines (one stream), plays it at ticks of dt with the
//       arm exactly on the reference, and prints "t q_d1..7 qd_d1..7 f1..3" per tick
#include "aris_controllers/core.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace aris_controllers;

static int checks = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    ++checks;                                                             \
    if (!(cond)) {                                                        \
      std::printf("FAIL line %d: %s\n", __LINE__, #cond);                 \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

static Sample make(uint32_t stream, double t, const Vec7& q, bool last = false) {
  Sample s;
  s.stream = stream;
  s.t = t;
  s.q = q;
  s.last = last;
  return s;
}

static Vec7 tick(Core& c, const Vec7& q, double dt = 0.001) {
  return c.update(q, Vec7::Zero(), Mat37::Zero(), Vec7::Zero(), dt);
}

// The joint stiffness and damping the law applies at J with the pen down on normal n (n = 0:
// pen up), probed column by column (the law is linear in e and ed).
static void probe(const Mat37& J, const Vec3& n, Mat7& Kp, Mat7& Dp, double& dn,
                  const Mat7& M = Mat7::Zero()) {
  Params p;
  p.tau_max.setConstant(1e12);
  p.tau_rate = 1e18;
  p.max_error = 1e9;
  Core c(p, 8);
  const Vec7 z = Vec7::Zero();
  c.activate(z);
  Sample s = make(1, 0.0, z, true);
  s.n = n;
  c.offer(s);
  c.update(z, z, J, z, 0.001, M);
  for (int i = 0; i < 7; ++i) {
    const Vec7 u = Vec7::Unit(i);
    Kp.col(i) = c.update(-u, z, J, z, 0.001, M);
    Dp.col(i) = c.update(z, -u, J, z, 0.001, M);
  }
  dn = c.status().d_normal;
}

static void pen_down_law() {
  const Params p;
  std::srand(3);
  for (int trial = 0; trial < 50; ++trial) {
    const Mat37 J = Mat37::Random() * 0.5;
    const Vec3 n = Vec3::Random().normalized();
    Mat7 Kp, Dp;
    double dn = 0.0;
    probe(J, n, Kp, Dp, dn);
    CHECK((Kp - Kp.transpose()).norm() < 1e-9 * Kp.norm());       // symmetric
    CHECK(Eigen::SelfAdjointEigenSolver<Mat7>(Kp).eigenvalues().minCoeff() > 0.0);
    Mat3 Kt, Kt2, Dt, Dt2;
    CHECK(tip_space(J, p.k, Kt) && tip_space(J, p.d, Dt));
    const Mat3 Kpt = (J * Kp.inverse() * J.transpose()).inverse();  // tip stiffness now
    const Mat3 P = Mat3::Identity() - n * n.transpose();
    const Mat3 want = P * Kt * P + p.k_normal * n * n.transpose();
    CHECK((Kpt - want).norm() < 1e-6 * want.norm());
    CHECK(std::abs(n.dot(Kpt * n) - p.k_normal) < 1e-6 * p.k_normal);
    CHECK(std::abs(1.0 / n.dot(Kpt.inverse() * n) - p.k_normal) < 1e-6 * p.k_normal);
    const double d_want = 2.0 * std::sqrt(p.k_normal * p.mass_normal);
    CHECK(std::abs(dn - d_want) < 1e-12);
    const Mat3 Dpt = (J * Dp.inverse() * J.transpose()).inverse();
    CHECK(std::abs(n.dot(Dpt * n) - d_want) < 1e-6 * d_want);
    CHECK(((P * Dpt * P) - (P * Dt * P)).norm() < 1e-6 * Dt.norm());
  }
  {  // pen up: plain joint impedance
    Mat7 Kp, Dp;
    double dn;
    probe(Mat37::Random(), Vec3::Zero(), Kp, Dp, dn);
    CHECK((Kp - Mat7(p.k.asDiagonal())).norm() < 1e-12 && (Dp - Mat7(p.d.asDiagonal())).norm() < 1e-12);
  }
  {  // with a mass matrix: critically damped for the arm's own mass along the normal
    const Mat37 J = Mat37::Random();
    const Vec3 n = Vec3::UnitZ();
    const Mat7 A = Mat7::Random();
    const Mat7 M = A * A.transpose() + Mat7::Identity();
    Mat7 Kp, Dp;
    double dn;
    probe(J, n, Kp, Dp, dn, M);
    const double m = 1.0 / n.dot(J * M.inverse() * J.transpose() * n);
    CHECK(std::abs(dn - 2.0 * std::sqrt(p.k_normal * m)) < 1e-9 * dn);
  }
}

static int probe_lines() {
  Mat37 J;
  Vec3 n;
  while (true) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 7; ++c)
        if (!(std::cin >> J(r, c))) return 0;
    for (int i = 0; i < 3; ++i) std::cin >> n(i);
    Mat7 Kp, Dp;
    double dn;
    probe(J, n, Kp, Dp, dn);
    for (int r = 0; r < 7; ++r)
      for (int c = 0; c < 7; ++c) std::printf("%.17g ", Kp(r, c));
    for (int r = 0; r < 7; ++r)
      for (int c = 0; c < 7; ++c) std::printf("%.17g ", Dp(r, c));
    std::printf("%.17g\n", dn);
  }
}

static void scenarios() {
  const Vec7 q0 = (Vec7() << 0.1, -0.2, 0.3, -1.5, 0.2, 1.4, 0.5).finished();
  Params p;

  {  // a stream is followed to its end and held there
    Core c(p, 64);
    c.activate(q0);
    Vec7 q1 = q0;
    q1(0) += 0.004;
    c.offer(make(1, 0.0, q0));
    c.offer(make(1, 0.004, q1, true));
    for (int i = 0; i < 10; ++i) tick(c, c.status().q_d);
    CHECK(c.status().done && !c.status().streaming);
    CHECK((c.status().q_d - q1).norm() < 1e-15);
    CHECK(c.status().hold == Hold::kNone);
  }
  {  // a stream that runs dry latches a hold after the starve timeout, and refuses streams
    Core c(p, 64);
    c.activate(q0);
    c.offer(make(1, 0.0, q0));
    c.offer(make(1, 0.001, q0));
    for (int i = 0; i < 15; ++i) tick(c, q0);
    CHECK(c.status().hold == Hold::kNone && c.status().streaming);  // inside the timeout
    for (int i = 0; i < 20; ++i) tick(c, q0);
    CHECK(c.status().starved && c.status().hold == Hold::kStarved);
    c.offer(make(2, 0.0, q0));
    CHECK(c.status().rejected == 2 && c.status().stream == 1);
    c.resume(q0);
    c.offer(make(3, 0.0, q0));
    CHECK(c.status().streaming && c.status().stream == 3);
  }
  {  // the start tolerance can be set for a job: 0.02 rad off is then accepted
    Core c(p, 64);
    c.set_start_tolerance(0.03);
    c.activate(q0);
    Vec7 off = q0;
    off(1) += 0.02;
    c.offer(make(9, 0.0, off));
    CHECK(c.status().streaming && c.status().stream == 9);
  }
  {  // a stream that does not start at the reference is refused, and stays refused
    Core c(p, 64);
    c.activate(q0);
    Vec7 far = q0;
    far(3) += 0.05;
    c.offer(make(4, 0.0, far));
    CHECK(c.status().rejected == 4 && !c.status().streaming);
    c.offer(make(4, 0.001, q0));
    CHECK(!c.status().streaming);
  }
  {  // a tracking error over the limit latches a hold where the arm is, force off
    Core c(p, 64);
    c.activate(q0);
    Sample a = make(1, 0.0, q0);
    a.f = Vec3(0, 0, 1.0);
    c.offer(a);
    Vec7 pushed = q0;
    pushed(2) += 0.06;
    tick(c, pushed);
    CHECK(c.status().hold == Hold::kTracking && c.status().error_joint == 2);
    CHECK((c.status().q_d - pushed).norm() < 1e-15 && c.status().f_ff.norm() == 0.0);
  }
  {  // the force is clamped to f_max, and the torque obeys the rate limit
    Core c(p, 64);
    c.activate(q0);
    Sample a = make(1, 0.0, q0, true);
    a.f = Vec3(0, 0, 50.0);
    c.offer(a);
    Mat37 J = Mat37::Zero();
    J(2, 1) = 1.0;  // joint 2 moves the tip along z by 1 m/rad
    Vec7 tau = c.update(q0, Vec7::Zero(), J, Vec7::Zero(), 0.001);
    CHECK(std::abs(tau(1) - p.tau_rate * 0.001) < 1e-12);  // rate-limited first step
    for (int i = 0; i < 20; ++i) tau = c.update(q0, Vec7::Zero(), J, Vec7::Zero(), 0.001);
    CHECK(std::abs(tau(1) - p.f_max) < 1e-12);               // clamped force, 1 m lever
  }
  {  // idle after a stream: the force is taken away after the timeout
    Core c(p, 64);
    c.activate(q0);
    Sample a = make(1, 0.0, q0, true);
    a.f = Vec3(0, 0, 1.0);
    c.offer(a);
    tick(c, q0);
    CHECK(c.status().done && c.status().f_ff.norm() == 1.0);
    for (int i = 0; i < 2100; ++i) tick(c, q0);
    CHECK(c.status().f_ff.norm() > 0.0 && c.status().f_ff.norm() < 1.0);
    for (int i = 0; i < 500; ++i) tick(c, q0);
    CHECK(c.status().f_ff.norm() == 0.0);
  }
  {  // the queue hands samples over in order and refuses when full
    SampleQueue qu(3);
    CHECK(qu.push(make(1, 0.0, q0)) && qu.push(make(1, 1.0, q0)) && qu.push(make(1, 2.0, q0)));
    CHECK(!qu.push(make(1, 3.0, q0)));
    Sample s;
    CHECK(qu.pop(s) && s.t == 0.0 && qu.pop(s) && s.t == 1.0 && qu.pop(s) && s.t == 2.0);
    CHECK(!qu.pop(s));
  }
  pen_down_law();
  std::printf("PASS %d\n", checks);
}

static int follow(double dt) {
  std::vector<Sample> in;
  Sample s;
  s.stream = 1;
  while (std::cin >> s.t) {
    for (int i = 0; i < 7; ++i) std::cin >> s.q(i);
    for (int i = 0; i < 7; ++i) std::cin >> s.qd(i);
    for (int i = 0; i < 3; ++i) std::cin >> s.f(i);
    in.push_back(s);
  }
  if (in.empty()) return 1;
  in.back().last = true;
  Params p;
  p.max_error = 1e9;
  Core c(p, in.size() + 1);
  SampleQueue qu(in.size());
  for (const auto& x : in) qu.push(x);
  c.activate(in.front().q);
  Sample x;
  while (qu.pop(x)) c.offer(x);
  do {
    tick(c, c.status().q_d, dt);
    const Status& st = c.status();
    std::printf("%.17g", st.t);
    for (int i = 0; i < 7; ++i) std::printf(" %.17g", st.q_d(i));
    for (int i = 0; i < 7; ++i) std::printf(" %.17g", st.qd_d(i));
    for (int i = 0; i < 3; ++i) std::printf(" %.17g", st.f_ff(i));
    std::printf("\n");
  } while (c.status().streaming);
  return 0;
}

int main(int argc, char** argv) {
  if (argc >= 3 && std::string(argv[1]) == "follow") return follow(std::atof(argv[2]));
  if (argc >= 2 && std::string(argv[1]) == "probe") return probe_lines();
  scenarios();
  return 0;
}
