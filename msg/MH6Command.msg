std_msgs/Header header

string hand
string source
bool dry_run
bool hardware_enabled
bool rate_limit_enabled

# 7 values: [u_thumb, u_index, u_middle, u_ring, u_little, u_h, u_v]
float32[7] low_dim

# 3 values: [thumbSide, littleSide, u_h]
float32[3] palm_norm

# 3 values: [P_opp, g, t]
float32[3] intent

int16[5] finger_positions
int16[3] palm_positions
uint16[3] palm_times
