from evdev import InputDevice, categorize, ecodes
dev = InputDevice('/dev/input/event0')
for event in dev.read_loop():
	print(event)

