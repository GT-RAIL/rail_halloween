#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from assistance_arbitrator.server import AssistanceArbitrationServer

def main():
    rospy.init_node('arbitrator')
    server = AssistanceArbitrationServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
