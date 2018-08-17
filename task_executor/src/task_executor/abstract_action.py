#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

import abc


class AbstractAction(object):
    """All actions are derived from this class"""

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self, locations, objects):
        """Initialize the databases. Really these should be their own thing"""
        raise NotImplementedError()

    @abc.abstractmethod
    def run(self, **params):
        """
        Run the action. This method returns a generator. So don't return;
        instead yield an empty dictionary of return values to keep executing.
        Yield a non-empty dictionary or raise StopIteration to stop the run
        """
        raise NotImplementedError()
