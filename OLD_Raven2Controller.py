import sys, termios
# Mode: run condition, home, manual mode, file mode
MODE = [True, False, False, False]

#
RECORD = [False, False]


def reset_mode():
	globals()
	# reset all modes
	MODE = [True, False, False, False]

def print_menu():
	print("Choose your mode: ")


def get_inputs():
	globals()
	print("RAVEN 2 Standardized Controller:")
	print("Enter 'h' for home")
	print("Enter 'm' for manual control")
	# add file valid here
	print("Enter 'f' for file control")
	print("Choose your mode:")
	while MODE[0]:
		input = sys.stdin.read(1)[0]
		termios.tcflush(sys.stdin, termios.TCIOFLUSH)
		if input == "h":
			reset_mode()
			print("Homing")
			MODE[1] = True
			print("Enter key to switch to control mode")

def run():
	globals()
	# main loop: running
	while MODE[0]:
		# home mode
		while MODE[1]:
			print("Home mode is activated")

		# manual mode
		while MODE[2]:
			print("Manual mode is activated")

		# file mode
		while MODE[3]:
			print("File mode is activated")


def main():
	pass


if __name__ == "__main__":
	main()
