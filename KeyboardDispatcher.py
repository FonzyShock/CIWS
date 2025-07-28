# !/usr/bin/python3
# KeyboardDispatcher.py created by Desktop at 7/27/2025

import sys
import termios
import tty
import threading
import select
import time

class KeyboardDispatcher:
    """
    A threaded keyboard listener that sends keystrokes to navigation and turret control subsystems.
    Supports simultaneous manual drive and turret aiming via unified key handler.
    """
    def __init__(self, nav_handler=None, trigger_handler=None):
        self.nav_handler = nav_handler      # function to call for navigation keys (e.g., w/a/s/d)
        self.trigger_handler = trigger_handler  # function to call for aiming/firing keys (e.g., i/j/k/l/space)
        self.running = True
        self._thread = threading.Thread(target=self._key_loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.running = False

    def _key_loop(self):
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
                    elif ch in ('i', 'j', 'k', 'l', '\r', 'q') and self.trigger_handler:
                        self.trigger_handler(ch)

                    # Exit command
                    if ch == 'q':
                        print("[Keyboard] 'q' pressed — exiting control mode.")
                        self.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# Example usage pattern:
if __name__ == "__main__":
    def mock_nav(ch):
        print(f"[Nav] Key: {ch}")

    def mock_trig(ch):
        print(f"[Trigger] Key: {ch}")

    kb = KeyboardDispatcher(nav_handler=mock_nav, trigger_handler=mock_trig)
    kb.start()

    try:
        while kb.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        kb.stop()
