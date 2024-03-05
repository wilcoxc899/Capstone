import keyboard
import time

def is_up_arrow_pressed():
    return keyboard.is_pressed("up")

def main():
    previous_state = False

    while True:
        current_state = is_up_arrow_pressed()

        if current_state != previous_state:
            print(f"Up Arrow {'Pressed' if current_state else 'Released'}")
            # Do something when the state changes, e.g., set a variable to True/False

        previous_state = current_state
        time.sleep(0.1)  # Adjust sleep duration as needed

if __name__ == "__main__":
    main()
