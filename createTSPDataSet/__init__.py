import os
import sys
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)
print("Add current path to sys.path: ", current_dir)
