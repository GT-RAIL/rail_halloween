#!/usr/bin/env python
# The operations that can be specified in the YAML

from __future__ import print_function, division

import rospy

from geometry_msgs.msg import PoseStamped


# The Operations

def assign(var_name, value):
    """
    Assigns the indicated `value` to a variable with name `var_name`

    Args:
        var_name (str) : Name of the variable to assign the value to
        value (any) : Value to assign to that variable
    Returns:
        A dictionary with { var_name: value }
    """
    return { var_name: value }
