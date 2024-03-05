from pynput import keyboard
import time

class ArrowKeyDetector:
    def __init__(self):
        self.is_up_arrow_pressed = False

    def on_press(self, key):
        if key == keyboard.Key.up:
            self.is_up_arrow_pressed = True
            print("Up Arrow Pressed")

    def on_release(self, key):
        if key == keyboard.Key.up:
            self.is_up_arrow_pressed = False
            print("Up Arrow Released")

    def run(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while True:
                time.sleep(1)

if __name__ == "__main__":
    detector = ArrowKeyDetector()
    detector.run()

