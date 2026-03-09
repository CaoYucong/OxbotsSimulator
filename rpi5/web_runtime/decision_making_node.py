#!/usr/bin/env python3
"""Decision-making runtime node for Raspberry Pi (non-ROS).

This directly reuses decision_making_ros/waypoints_cruise.py by calling its main().
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


class DecisionMakingNode:
    def __init__(self, project_root: Path) -> None:
        self.project_root = project_root
        self.script_dir = project_root / "decision_making_ros"

        if not self.script_dir.exists():
            raise FileNotFoundError(f"decision_making_ros folder not found: {self.script_dir}")

        # Ensure local import resolves to the workspace copy.
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))

        from decision_making_ros import waypoints_cruise as planner  # pylint: disable=import-outside-toplevel

        self._planner = planner
        os.chdir(self.script_dir)
        print(f"[decision_making] using script: {self.script_dir / 'waypoints_cruise.py'}")

    def tick(self) -> None:
        old_argv = sys.argv
        old_cwd = os.getcwd()
        try:
            sys.argv = ["waypoints_cruise.py"]
            os.chdir(self.script_dir)
            code = int(self._planner.main())
            if code not in (0,):
                print(f"[decision_making] script returned non-zero code: {code}")
        except Exception as exc:
            print(f"[decision_making] tick failed: {exc}")
        finally:
            os.chdir(old_cwd)
            sys.argv = old_argv
