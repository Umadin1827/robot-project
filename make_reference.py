"""Create data/pythonrobotics_reference.csv by running the ORIGINAL source code.

Run from the repository root:   python scripts/make_reference.py
It downloads pure_pursuit.py from GitHub, checks it runs, and records the
trajectory of its main() example (without any plotting).  The sha256 of the
downloaded file is printed so that you can see which version was used.
"""
import hashlib
import importlib.util
import math
import sys
import tempfile
import types
import urllib.request
from pathlib import Path

import numpy as np

URL = ("https://raw.githubusercontent.com/AtsushiSakai/PythonRobotics/"
       "master/PathTracking/pure_pursuit/pure_pursuit.py")
OUT = Path(__file__).resolve().parent.parent / "data" / "pythonrobotics_reference.csv"

code = urllib.request.urlopen(URL, timeout=30).read()
print("sha256 of source:", hashlib.sha256(code).hexdigest())

# the source imports utils.angle.angle_mod from the PythonRobotics repo: give it a tiny stand-in
utils = types.ModuleType("utils"); angle = types.ModuleType("utils.angle")
angle.angle_mod = lambda x: (x + math.pi) % (2 * math.pi) - math.pi
sys.modules["utils"], sys.modules["utils.angle"] = utils, angle

with tempfile.TemporaryDirectory() as tmp:
    f = Path(tmp) / "pure_pursuit_source.py"
    f.write_bytes(code)
    spec = importlib.util.spec_from_file_location("pp_source", f)
    pp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pp)

# same loop as main() in the source, but we keep the states
cx = np.arange(0, 50, 0.5)
cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
target_speed = 10.0 / 3.6
state = pp.State(x=0.0, y=-3.0, yaw=0.0, v=0.0)
course = pp.TargetCourse(cx, cy)
target_ind, _ = course.search_target_index(state)
t, rows, last = 0.0, [[0.0, state.x, state.y, state.yaw, state.v, 0.0]], len(cx) - 1
while 100.0 >= t and last > target_ind:
    a = pp.proportional_control(target_speed, state.v)
    delta, target_ind = pp.pure_pursuit_steer_control(state, course, target_ind)
    state.update(a, delta)
    t += pp.dt
    rows.append([t, state.x, state.y, state.yaw, state.v, delta])

OUT.parent.mkdir(exist_ok=True)
np.savetxt(OUT, np.array(rows), delimiter=",", header="t,x,y,yaw,v,steer", comments="")
print("saved", OUT, "rows:", len(rows))
