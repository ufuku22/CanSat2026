from gpiozero import OutputDevice


GPIO_PIN = 24


def main():
    output = OutputDevice(GPIO_PIN, active_high=True, initial_value=False)
    is_on = False

    try:
        print("GPIO24: OFF")
        print("Press Enter to toggle GPIO24. Press Ctrl+C to stop.")

        while True:
            input()
            is_on = not is_on
            output.value = is_on
            print(f"GPIO24: {'ON' if is_on else 'OFF'}")
    except KeyboardInterrupt:
        print()
    finally:
        output.off()
        output.close()
        print("GPIO24: OFF")


if __name__ == "__main__":
    main()
