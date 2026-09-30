# What aris_joint_impedance_controller is doing, published at `status_rate` Hz.

builtin_interfaces/Time stamp
uint32 stream              # the stream being, or last, followed
uint32 rejected            # the last stream refused
float64 t                  # that stream's clock, s
bool streaming
bool done                  # the stream reached its final sample and holds there
bool starved               # the stream ran dry before its final sample (latches a hold)
bool holding               # a latched hold: stands still, refuses streams until ~/resume
string reason              # why it holds ("" when not holding)
int32 error_joint          # 0-based joint that tripped the tracking limit, -1 if none
bool pen_down              # the soft spring along the paper normal is on
float64 d_normal           # N s/m, the damping along the normal this tick
float64[7] q_d
float64[7] qd_d
float64[7] tracking_error  # q_d - q, rad
float64[3] f_ff            # N, base frame, the force fed forward now
float64[3] force           # N, base frame: the robot's estimate of the external force at
                           # its end effector (O_F_ext_hat_K), zero without the model
float64[7] tau             # Nm, the torque commanded (gravity is added by the robot)
