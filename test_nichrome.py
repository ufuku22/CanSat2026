import time

import nichrome_SW


PULSE_SECONDS = 5.0


input("Press Enter to start nichrome wire ON...")

nichrome_SW.setup()

try:
    nichrome_SW.nichrome_on()
    time.sleep(PULSE_SECONDS)
finally:
    nichrome_SW.cleanup()
