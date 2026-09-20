"""
MicroPython boot.py — IoMT RPM Node
Runs once on power-up before main.py.
"""

import esp
esp.osdebug(None)  # Disable OS-level debug output on UART

import gc
gc.collect()
