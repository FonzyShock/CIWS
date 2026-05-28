# !/usr/bin/python3
# KeyboardDispatcher.py created by Desktop at 7/27/2025

import sys
import termios
import tty
import threading
import select

class KeyboardDispatcher(threading.Thread):
    """
    A threaded keyboard listener that sends keystrokes to navigation and turret control subsystems.
    Supports simultaneous manual drive and turret aiming via unified key handler.
    """

    def __init__(self, nav_handler=None, trigger_handler=None, quit_handler=None):
        super().__init__(daemon=True)
        self.nav_handler = nav_handler
        self.trigger_handler = trigger_handler
        self.quit_handler = quit_handler
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
                    if ch in ('w', 'a', 's', 'd') and self.nav_handler:
                        self.nav_handler(ch)

                    # Dispatch to trigger if it's a turret key
                    elif ch in ('i', 'j', 'k', 'l', 'f') and self.trigger_handler:
                        self.trigger_handler(ch)

                    # Exit command
                    # Exit command — trigger full system shutdown
                    if ch == 'q':
                        print("[Keyboard] 'q' pressed — shutting down CIWS.")
                        self.stop()
                        if self.quit_handler:
                            self.quit_handler()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
