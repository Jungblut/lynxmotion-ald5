lynxmotion-AL5D
===============

Python interface for the [Lynxmotion AL5D](https://www.lynxmotion.com/) robotic arm when it is driven by a Lynxmotion SSC-32u USB servo controller (serial over USB).

The high-level `AL5D` class maps joint angles to servo pulses, provides inverse kinematics for Cartesian moves, and batches coordinated motions where the API uses a move group. The lower-level `SSC32` class sends the same text commands the SSC-32 family expects, if you need direct servo control.

This project was developed over 10 years ago by @cberner in [github.com/cberner/lynxmotion](https://github.com/cberner/lynxmotion).

Quick examples
==============

Connect, home the arm, and wait until the motion finishes:

```python
import al5d

arm = al5d.AL5D("/dev/ttyUSB0")  # adjust to your serial device
arm.init()
arm.wait_for_move()
```

Move the gripper tip to a point in space (meters). The fourth argument is `phi`, the wrist angle in the vertical plane (radians), relative to the horizontal `xy` plane:

```python
import math
import al5d

arm = al5d.AL5D("/dev/ttyUSB0")

arm.move(x=0.20, y=0.0, z=0.18, phi=0.0)
arm.wait_for_move()
arm.gripper(100)   # 100 = fully closed, 0 = open
```

Drive joints directly by angle (radians). Ranges are documented on each method in `al5d/al5d.py` (base, shoulder, elbow, wrist, wrist rotate, gripper):

```python
import math
import al5d

arm = al5d.AL5D("/dev/ttyUSB0")

arm.base(math.pi / 2)
arm.shoulder(0)
arm.elbow(math.pi / 6)
arm.wrist(0)
arm.wrist_rotate(math.pi / 4)
arm.gripper(40)
```

Talk to the SSC-32u with the thin `SSC32` wrapper (pulse width, speed, optional time). Use a `with` block to send a grouped move so all listed servos start together:

```python
import ssc32

board = ssc32.SSC32("/dev/ttyUSB0")
print(board.version())

with board.move_group():
    board.move(0, 1500, speed=100)  # servo 0, pulse width, optional S/T
```

Setup
=====

Install dependencies:

```bash
uv sync
```

Find the serial device (often `/dev/ttyUSB0` on Linux). After connecting the SSC-32u over USB, check `dmesg` or list `/dev` for a new `ttyUSB` or `ttyACM` node and use that path in `AL5D(...)`.

Sanity check with the bundled script (clears the workspace of obstacles first):

```bash
uv run al5d/test.py
```

Notes
=====

- Cartesian `move(x, y, z, phi)` uses `Kinematics` (link lengths in meters). Pass `AL5D(..., kinematics=Kinematics(...))` to tune your hardware, or rely on `Kinematics()` defaults.
- `AL5D` is built for a specific arm configuration; interpolation comments in the code call out where behavior is tuned to one physical setup.
