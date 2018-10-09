# A message type to provide an execution trace

# A header indicating the time of the trace update
time stamp

# The name of the step that this is the update of
string name

# The actionlib_msgs/GoalStatus status of the component
uint8 status

# Any other context information to be included in the trace. This is a pickled byte array
string context
