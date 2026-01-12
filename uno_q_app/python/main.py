#!/usr/bin/env python3
"""
Entry point for the MeasurePi UNO Q app.

This file simply executes the bridge_mqtt.py script from the repository root.
"""

import runpy
import os

# Determine path relative to this file (two levels up to repo root)
base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
script_path = os.path.join(base_dir, 'bridge_mqtt.py')

# Execute the script in its own namespace as __main__
runpy.run_path(script_path, run_name="__main__")
