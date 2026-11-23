# Pure Pursuit Path Following for a Differential Drive Robot

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Umadin1827/robot-project/blob/main/lecture.ipynb)

**Final assignment, Scientific Programming for Physics: one-hour notebook lecture.**

## Summary
I simulate a two-wheeled (differential drive) robot with DC motors that has to follow a spiral path.
In my first notebook the robot drove from waypoint to waypoint with a simple P controller. Here I replace it with **pure pursuit**: the robot looks a distance `Ld` ahead on the path, computes the curvature `kappa = 2 sin(alpha) / Ld` of the circle to that target point, and turns it into voltages for the left and right motor.
I check the code with four physics tests, reproduce the pure pursuit example of the PythonRobotics library (my own re-implementation gives the same trajectory) and compare the new controller with the old one on the spiral (cross-track error and time).
The lecture is built as 9 timed sections (60 min), with an audience exercise (easy / medium / stretch).

**Route:** A (reproduction of a published method / reference code).

## How to run
**Colab (recommended):** click the badge above, then *Runtime → Run all*. The first cell clones this repository and checks the packages. It takes about 1 to 2 minutes.

**Locally:**
```bash
git clone https://github.com/Umadin1827/robot-project.git
cd robot-project
pip install -r requirements.txt
jupyter notebook lecture.ipynb        # then Run all
python -m pytest -q tests             # automatic tests
```
No manual steps, no absolute paths, all random numbers use a seed (`SEED = 0`), the whole notebook runs in well under 10 minutes (about 15 s on my test machine; the optional animation can be switched off with `SHOW_ANIMATION = False`).
If a different version of numpy/scipy/matplotlib is installed (for example on Colab), the notebook prints the difference. I tested with the versions in `requirements.txt`.

## Files
| file | content |
|---|---|
| `lecture.ipynb` | the lecture notebook (outputs cleared) |
| `lecture_executed.html` | the same notebook with all outputs (backup, for people without Python) |
| `solutions.ipynb` | solutions of the audience exercises |
| `src/` | helper code: `motor.py`, `robot.py`, `controllers.py`, `paths.py`, `metrics.py`, `plotting.py`, `bicycle.py` |
| `tests/` | pytest tests (the same checks are in the notebook as `assert`) |
| `data/pythonrobotics_reference.csv` | trajectory of the original PythonRobotics example (made with `scripts/make_reference.py`) |
| `scripts/make_reference.py` | downloads the original source file and records its trajectory |
| `REFLECTION.md` | one page reflection |

## Sources and licences
- R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01, Carnegie Mellon University, 1992.
- A. Sakai et al., *PythonRobotics: a Python code collection of robotics algorithms*, arXiv:1808.10703, 2018. Code: <https://github.com/AtsushiSakai/PythonRobotics> (`PathTracking/pure_pursuit/pure_pursuit.py`), **MIT licence**. I did not copy the file into this repository: `src/bicycle.py` is my own re-implementation of its algorithm with the same parameters, and `scripts/make_reference.py` downloads the original file at run time to create the reference trajectory in `data/`. That csv file contains only numbers produced by running the original code.
- Motor model, robot kinematics, the waypoint controller and the spiral path come from my own first notebook "Differential drive robot path following".
- My own code in this repository is under the MIT licence (see `LICENSE`).
- Snider, *Automatic Steering Methods for Autonomous Automobile Path Tracking*, CMU-RI-TR-09-08, 2009 (further reading only).

## AI assistant declaration
I used the AI assistant **Claude (Anthropic)** for this project:
- **Notebook and structure:** planning the 60-minute lecture structure, and writing the first version of `lecture.ipynb` and `solutions.ipynb` (text and code cells).
- **Code:** writing the pure pursuit controller (`src/controllers.py`: `find_target_index`, `curvature`, `wheel_speeds`, `speed_to_voltage`, `PurePursuit`), the fast exact motor step, `src/bicycle.py`, `src/metrics.py`, `src/plotting.py`, `scripts/make_reference.py` and the tests in `tests/`.
- **Text:** first drafts of this README and of `REFLECTION.md`.

Not written with AI: the original DC motor ODE `dc_motor`, the kinematics formulas (`update_pose`), the waypoint P controller and the spiral path. They come from my first notebook, which is my own work.
I ran all code, and I checked the results and explanations myself.
