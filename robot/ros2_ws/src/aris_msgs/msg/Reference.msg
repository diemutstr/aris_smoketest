# A chunk of reference samples for aris_joint_impedance_controller: consecutive samples of one
# stream.  Between two samples the reference is the cubic matching q and qd at both; the force
# is interpolated linearly.  N samples; arrays are row-major (sample after sample).
#
# A stream id larger than the controller's current one starts a new stream.  The controller's
# clock for the stream starts at t[0] when the first chunk arrives.  The first sample must be
# within the controller's start tolerance of where it holds, or the stream is refused.

uint32 stream
bool last            # the stream ends with the final sample of this chunk: hold there
float64[] t          # N, seconds on the stream clock, increasing
float64[] q          # 7N, rad
float64[] qd         # 7N, rad/s
float64[] f          # 3N, N, base frame: the force the arm applies at the pen tip
float64[] n          # 3N, the paper normal (base frame, unit) while the pen is down, zeros
                     # while it is up; empty = pen up throughout.  Pen down switches on the
                     # soft spring k_normal along it in place of the joint springs' stiffness
