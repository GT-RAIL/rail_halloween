#!/usr/bin/env python
# This is soundplay_node v2 where:
#   a) We don't keep pointers to the sinks around (so that we can clean them).
#   b) Conversely, we also (re)check to see if the wav file is available so that
#       we don't cause the alsa system to crash
#   c) We only present the action interface to playing sounds
#   d) We use a full ActionServer; not a simple one
#   e) We don't output diagnostics

from __future__ import print_function, division

import os
import psutil
import threading
import Queue

import rospy
import actionlib

from sound_play.msg import (SoundRequest, SoundRequestAction,
                            SoundRequestFeedback, SoundRequestResult)


# The class that encapsulates the action server to play sound requests

class SoundServer(object):
    """
    This is the server that serves sound requests coming in and plays them as
    specified. It does NOT (yet) implement all the functionality available to
    ROS's soundplay_node, or some other soundplay_nodes that I have seen.

    Specifically, this node:
    1. Does not (yet) honour loop requests.
    2. Does not (yet) perform text-to-speech. We assume that has been done by
        the client that is in this same package.
    3. Does not honour the predefined sound files that come with sound_play
    4. Does not allow the specification of sound priorities, overriding. If
        there are multiple sound requests that come in at the same time, all are
        played.
    """

    DEFAULT_SOUNDPLAY_COMMAND = "aplay"
    SOUND_SERVER_NAME = "sound_server"

    def __init__(self):
        # Managing the sound requests that come into the action server
        self._current_sounds = Queue()
        self._sound_manager_thread = threading.Thread(target=self._monitor_sounds)

        # The action server
        self._server = actionlib.ActionServer(
            SoundServer.SOUND_SERVER_NAME,
            SoundRequestAction,
            goal_cb=self.on_goal,
            auto_start=False
        )
