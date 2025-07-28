# !/usr/bin/python3
# KeyboardDispatcher.py created by Desktop at 7/27/2025

import sys
import termios
import tty
import threading
import select

class KeyboardDispatcher:
    """
    A threaded keyboard listener that sends keystrokes to navigation and turret control subsystems.
    Supports simultaneous manual drive and turret aiming via unified key handler.
    """

    def __init__(self, nav_handler=None, trigger_handler=None):
        super().__init__(daemon=True)
        self.nav_handler = nav_handler  # function to call for navigation keys (e.g., w/a/s/d)
        self.trigger_handler = trigger_handler  # function to call for aiming/firing keys (e.g., i/j/k/l/space)
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            print("[Keyboard] Listening for key input...")
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)

                    # Dispatch to navigation if it's a drive key
                    if ch in ('w', 'a', 's', 'd', 'q') and self.nav_handler:
                        self.nav_handler(ch)

                    # Dispatch to trigger if it's a turret key
                    elif ch in ('i', 'j', 'k', 'l', ' ', '\r', 'q') and self.trigger_handler:
                        self.trigger_handler(ch)

                    # Exit command
                    if ch == 'q':
                        print("[Keyboard] 'q' pressed — exiting control mode.")
                        self.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
