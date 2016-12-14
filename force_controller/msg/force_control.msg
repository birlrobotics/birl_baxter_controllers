#Number of Controllers
int32 num_ctrls

# Name of the type(s) of Controller(s): force or moment
string[] type

# Desired force/moment (3D)
geometry_msgs/Vector3[] desired

# Gains force/moment (3D) this is optional if not given use default
geometry_msgs/Vector3[] gains
