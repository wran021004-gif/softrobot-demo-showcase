"""Unified native viewer, saved states only."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from visualization.mujoco_renderer import main
if __name__=='__main__':sys.argv.insert(1,'m8');main()
