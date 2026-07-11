# Shared state message for each robot in the swarm.

# Identity
string robot_name
builtin_interfaces/Time stamp

# Robot pose and motion
float64 x
float64 y
float64 yaw
float64 linear_speed

# Leader selection
float64 leader_score

# Current swarm role
string role
string leader_id
bool active

# Relay-tree group information
string group_id
string parent_group_id
string parent_relay_id

# Relay-tree branch information
float64 assigned_heading_deg
int32 branch_depth

# Relay-tree role flags
bool is_relay
bool is_group_leader