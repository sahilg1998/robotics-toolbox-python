#!/usr/bin/env python3

# a simple Robotics Toolbox "shell", runs Python3 and loads in NumPy, RTB, SMTB
# 
# Run it from the shell
#  % rtb.py
#
# or setup an alias
#
#  alias rtb=PATH/rtb.py   # sh/bash
#  alias rtb PATH/rtb.py   # csh/tcsh
#
# % rtb

# import stuff
import argparse
from math import pi              # lgtm [py/unused-import]
import numpy as np
import matplotlib as plt         # lgtm [py/unused-import]
from roboticstoolbox import *   # lgtm [py/unused-import]
from spatialmath import *        # lgtm [py/polluting-import]
from spatialmath.base import *   # lgtm [py/polluting-import]


# setup defaults
np.set_printoptions(linewidth=120, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})
SE3._ansimatrix = True

parser = argparse.ArgumentParser('Robotics Toolbox shell')
parser.add_argument('script', default=None, nargs='?',
    help='specify script to run')
parser.add_argument('--backend', '-b', default=None,
    help='specify graphics frontend')
args = parser.parse_args()

if args.backend is not None:
    print(f"Using matplotlub backend {args.backend}")
    plt.use(args.backend)

# load some models
puma = models.DH.Puma560()
panda = models.DH.Panda()

# print the banner
# https://patorjk.com/software/taag/#p=display&f=Cybermedium&t=Robotics%20Toolbox%0A
print(r"""____ ____ ___  ____ ___ _ ____ ____    ___ ____ ____ _    ___  ____ _  _
|__/ |  | |__] |  |  |  | |    [__      |  |  | |  | |    |__] |  |  \/
|  \ |__| |__] |__|  |  | |___ ___]     |  |__| |__| |___ |__] |__| _/\_

for Python

from roboticstoolbox import *
from spatialmath import *

""")

if args.script is not None:
    path = Path(args.script)
    if not path.exists():
        raise ValueError(f"script does not exist: {args.script}")
    exec(path.read_text())

# drop into IPython
import IPython
IPython.embed()