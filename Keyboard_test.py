import keyboard
import time

class ArrowKeyDetector:
    def __init__(self):
        self.is_up_arrow_pressed = False

    def on_key_event(self, e):
        if e.event_type == keyboard.KEY_DOWN and e.name == 'up':
            if not self.is_up_arrow_pressed:
                self.is_up_arrow_pressed = True
                print("Up Arrow Pressed")
        elif e.event_type == keyboard.KEY_UP and e.name == 'up':
            if self.is_up_arrow_pressed:
                self.is_up_arrow_pressed = False
                print("Up Arrow Released")

    def run(self):
        keyboard.hook(self.on_key_event)
        keyboard.wait()

if __name__ == "__main__":
    detector = ArrowKeyDetector()
    detector.run()

