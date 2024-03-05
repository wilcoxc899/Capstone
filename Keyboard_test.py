import keyboard
import time

class ArrowKeyDetector:
    def __init__(self):
        self.is_up_arrow_pressed = False

    def run(self):
        while True:
            if keyboard.is_pressed('up'):
                if not self.is_up_arrow_pressed:
                    self.is_up_arrow_pressed = True
                    print("Up Arrow Pressed")
            else:
                if self.is_up_arrow_pressed:
                    self.is_up_arrow_pressed = False
                    print("Up Arrow Released")

            time.sleep(0.1)

if __name__ == "__main__":
    detector = ArrowKeyDetector()
    detector.run()
