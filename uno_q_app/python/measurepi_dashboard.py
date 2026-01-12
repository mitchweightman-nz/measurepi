#!/usr/bin/env python3
"""
Flask dashboard wrapper for the UNO Q.

This file executes the top-level measurepi_dashboard.py module from the
repository root so that the Flask API can be run manually on the UNO Q.
"""

import runpy
import os

# Determine the repository root relative to this file (two levels up)
base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
module_path = os.path.join(base_dir, 'measurepi_dashboard.py')

# Execute the module as __main__
runpy.run_path(module_path, run_name="__main__")
