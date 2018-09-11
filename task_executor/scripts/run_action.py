#!/usr/bin/env python
# Runs an action

import sys
import json
import argparse

import rospy

from actionlib_msgs.msg import GoalStatus
from task_executor.actions import get_default_actions


def goal_status_from_code(status):
    mapping = {
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.ABORTED: "ABORTED",
    }
    return mapping[status]


def main():
    # Initialize the node
    rospy.init_node('action_client')

    # Instantiate the actions. But initialize only the ones we need
    actions = get_default_actions()

    # Create the argparser
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='action')
    for key, action in actions.registry.iteritems():
        action_parser = subparsers.add_parser(key, help="Action: {}".format(key))
        action_parser.add_argument('params', help="params as JSON to the action")

    # Then parse the arguments
    args = parser.parse_args()

    # Initialize the action
    action = actions[args.action]
    action.init(args.action)

    # Read the params
    def convert_params(d):
        new_d = {}
        for k, v in d.iteritems():
            if isinstance(k, unicode):
                k = str(k)
            new_d[k] = v

            if isinstance(v, unicode):
                new_d[k] = str(v)
            elif isinstance(v, dict):
                new_d[k] = convert_params(v)

        return new_d

    params = json.loads(args.params)
    params = convert_params(params)

    # Run the action and return the value
    status, variables = action(**params)
    rospy.loginfo("Status: {}. Variables: {}".format(
        goal_status_from_code(status), variables
    ))


if __name__ == '__main__':
    main()
