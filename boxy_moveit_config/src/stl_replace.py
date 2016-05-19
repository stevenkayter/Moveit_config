#!/usr/bin/env python
from glob import glob
from subprocess import call
import rospkg

"""
This script rewrites every .stl in the iai_robots repo.
"""

def replace_stl(dir):
    """
    Converts every .stl in dir and its subdirectories into a new .stl.
    :param dir: string
        target directory
    """
    files = glob(dir+"*.stl")
    for f in files:
        call(["ctmconv", f, f])
    dirs = glob(dir+"*/")
    for d in dirs:
        replace_stl(d)

rospack = rospkg.RosPack()
dir = rospack.get_path("iai_boxy_base")+"/../"
print dir
replace_stl(dir)
