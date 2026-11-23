# Reflection

**Topic:** pure pursuit path following for my differential drive robot (Route A, reproduction of a published method).

## What reproduced
- **The PythonRobotics pure pursuit example.** I wrote the algorithm again in `src/bicycle.py` and compared it with the trajectory of the original code (`data/pythonrobotics_reference.csv`). The positions are identical: the largest difference is printed in section 6.1 and it is far below my criterion of 1e-9 m. The steering angles differ only by rounding (about 1e-15 rad).
- **The physics of the method.** Four checks pass in the notebook and in `tests/`: the fast exact motor formula equals `solve_ivp`; the pure pursuit formula gives exactly 1/R on a circle; constant wheel voltages give the circle radius from the kinematics; the robot converges onto a straight line.
- **The improvement for my robot.** On the same spiral, pure pursuit (Ld = 0.3 m, v = 0.25 m/s) is better than my first waypoint controller: the RMS cross-track error went from 3.8 cm to 1.8 cm and the time from 125 s to 54 s (numbers from the executed notebook, section 6.2).

## What did not reproduce / what I could not show
- The source is a code example, not a paper with a figure and numbers, so my comparison is "my code vs the original code". It proves that I copied the algorithm correctly, not that the algorithm is good. The correctness of the formula comes from the analytic checks.
- Everything is a **simulation**. I did not test on the real robot, so wheel slip, sensor delay and the real motor differences are not included.
- The biggest error of pure pursuit is in the first seconds (about 11 cm): the robot starts looking along x, but the path starts going up. Pure pursuit cannot fix a bad start quickly.
- I changed one detail of my first notebook: there the control signal of the previous step was given to the motor (one step delay). Now the current signal is used. With a step of 0.01 s this is a small difference, but the numbers of the waypoint controller are therefore not exactly the same as in the first notebook.
- For speed I replaced `solve_ivp` in every step by the exact solution of the linear motor equation. I verified this (check 1), but it is a change.

## What I would change
- Make the speed depend on the curvature: the robot should slow down in the tight loops (the tightest turn is only about 1.7 times the wheel distance).
- Use the speed dependent look-ahead of the source (`Ld = k*v + Lfc`) also for the robot.
- Filter the position before using it, because exercise 3 shows that a small Ld reacts strongly to sensor noise.
- Test on the real robot and compare with other path trackers (for example Stanley or LQR).
- Spend less time on animation code in the lecture and more on letting the audience change parameters.
