"""Quick smoke test to verify Python works in the Roads project."""

import sys
import os
import platform

print(f"Python:   {sys.version}")
print(f"Platform: {platform.platform()}")
print(f"CWD:      {os.getcwd()}")
print(f"Script:   {os.path.abspath(__file__)}")
print("Python is working!")
