# One object found on the table by the camera (layer-1 geometric perception).

# Stable id while the object stays in view, e.g. "obj_3". Also the MoveIt
# collision-object id.
string id

# "box" or "cylinder"
string shape
uint8 SHAPE_BOX=0
uint8 SHAPE_CYLINDER=1
uint8 shape_type

# Colour name from the median RGB of the object's pixels ("green", "cyan", ...)
string color
std_msgs/ColorRGBA rgb

# Object centre and orientation (z up; for a box, x along the longer side).
geometry_msgs/PoseStamped pose

# box: [x, y, z] side lengths in m; cylinder: [height, radius] (MoveIt order)
float64[] dimensions

# Width the gripper jaws must span (box: shorter side, cylinder: diameter), m.
float64 grasp_width
bool graspable

uint32 num_points
