#!/usr/bin/env python3
"""Legacy interactive console and keyboard diagnostics."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import debug_main


if __name__ == '__main__':
    debug_main()
