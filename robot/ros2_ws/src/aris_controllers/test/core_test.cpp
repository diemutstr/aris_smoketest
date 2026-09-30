// Tests of the controller core without ROS.  Build and run (robot/tests/test_controller_core.py
// does this):
//   g++ -std=c++17 -O2 -I<eigen3> -I../include core_test.cpp -o core_test
//   ./core_test                      scenario checks; prints "PASS n" or the first failure
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
  scenarios();
  return 0;
}
