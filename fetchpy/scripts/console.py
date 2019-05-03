#!/usr/bin/env python

"""
Provides a simple console for using Fetchpy

sudo apt-get install python-pip python-ipython

Type 'exit' to close IPython and this script.
"""

import numpy
import sys

import fetchpy


if __name__ == "__main__":

    world, robot = fetchpy.initialize()

    import IPython
    IPython.embed()

