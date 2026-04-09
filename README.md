lynxmotion-al5d
===============

Python interface for Lynxmotion AL5D.

Setup
=====

Pull the dependencies using: `uv sync`

Check what device it was installed under, most likely /dev/ttyUSB0, by running `dmesg` or checking what new device showed up in /dev.

Confirm that it's working by running `uv run al5d/test.py`

