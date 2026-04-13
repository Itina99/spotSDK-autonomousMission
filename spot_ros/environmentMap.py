import os
import sys

# Reuse the existing, hardware-agnostic EnvironmentMap implementation.
# This keeps ROS-specific code minimal while preserving all map logic.
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)

from environmentMap import EnvironmentMap  # noqa: F401
