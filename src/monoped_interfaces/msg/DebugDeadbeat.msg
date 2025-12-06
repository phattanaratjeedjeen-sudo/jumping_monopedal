# Debug message for deadbeat controller state
# Contains all relevant state information for logging and plotting

float64 zb              # body height (m)
float64 zb_dot          # body velocity (m/s)
float64 zf              # foot height (m)
float64 hk              # actual height of current hop (m), NaN if not set
float64 hc              # apex height of current hop given to controller (m)
float64 hd              # desired height to reach at apex of next hop (m)
float64 u               # control input (J)
float64 effort          # effort command (N)
int8 state_code         # state: air=0, compress=1, touchdown=2, rebound=3, unknown=-1
