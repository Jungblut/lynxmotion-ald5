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

LeRobot
=======

The `lerobot_robot_al5d` package adds a [LeRobot](https://huggingface.co/docs/lerobot) robot, so the arm works with
`lerobot-teleoperate`, `lerobot-record` and friends. The plain driver stays usable without it: `import al5d` never
imports lerobot, and `import lerobot_robot_al5d` needs the `lerobot` extra.

```bash
uv sync                      # development: includes lerobot
pip install "lerobot_robot_al5d[lerobot]"   # elsewhere
```

lerobot finds the plugin by its installed distribution name (`lerobot_robot_*`), so the robot is available as soon as it is
installed next to lerobot:

```bash
uv run lerobot-teleoperate --robot.type=lynxmotion_al5d --robot.port=/dev/ttyUSB0 ...
```

Or from Python:

```python
from lerobot_robot_al5d import LynxmotionAL5D, LynxmotionAL5DConfig

robot = LynxmotionAL5D(LynxmotionAL5DConfig(port="/dev/ttyUSB0"))
robot.connect()  # homes the arm first
robot.send_action({"base.pos": 90.0, "elbow.pos": 30.0, "gripper.pos": 40.0})
print(robot.get_observation())
robot.disconnect()
```

- Actions and observations are `<joint>.pos` for `base`, `shoulder`, `elbow`, `wrist`, `wrist_rotate` and `gripper`. Joints are in
  **degrees** using the driver's own conventions and ranges (see `al5d/al5d.py`); `gripper` is 0 (open) to 100 (closed).
  Out-of-range actions are clipped, and `send_action` returns what was actually sent.
- The SSC-32 drives hobby servos without feedback, so **observations are the last commanded target, not a measurement**.
  `connect()` therefore homes the arm by default (`--robot.home_on_connect=false` to skip).
- `--robot.speed` (default 500) is the SSC-32 speed in µs/s applied to every move; lower is slower.
- No calibration is needed: pulse widths are mapped by fixed constants in the driver.

Setup
=====

Install dependencies (Python 3.12 is pinned in `.python-version`, because lerobot pins `numpy<2.3`, which has no Python 3.14
wheels yet):

```bash
uv sync
```

Find the serial device (often `/dev/ttyUSB0` on Linux). After connecting the SSC-32u over USB, check `dmesg` or list `/dev` for a new `ttyUSB` or `ttyACM` node and use that path in `AL5D(...)`.

Sanity check with the bundled script (clears the workspace of obstacles first):

```bash
uv run python -m al5d.test
```

Notes
=====

- Cartesian `move(x, y, z, phi)` uses `Kinematics` (link lengths in meters). Pass `AL5D(..., kinematics=Kinematics(...))` to tune your hardware, or rely on `Kinematics()` defaults.
- `AL5D` is built for a specific arm configuration; interpolation comments in the code call out where behavior is tuned to one physical setup.
