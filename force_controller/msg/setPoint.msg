#Number of Controllers
int32 num_ctrls

# Type of dominant controller
string domType

# Desired force/moment (3D) for dominant controller
geometry_msgs/Vector3[] domDes

# Gains force/moment (3D) for dominant controller
geometry_msgs/Vector3[] domGains

# Type of subordinate controller
string subType

# Desired force/moment (3D) for subordinate controller
geometry_msgs/Vector3[] subDes

# Gains force/moment (3D) for subordinate controller
geometry_msgs/Vector3[] subGains
