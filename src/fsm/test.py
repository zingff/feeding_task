import rospy

# Define ANSI color codes
class AnsiColorCodes:
    END = '\033[0m'
    BLACK = '\033[0;30m'
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[0;33m'
    BLUE = '\033[0;34m'
    PURPLE = '\033[0;35m'
    CYAN = '\033[0;36m'
    WHITE = '\033[0;37m'
    BOLD_BLACK = '\033[1;30m'
    BOLD_RED = '\033[1;31m'
    BOLD_GREEN = '\033[1;32m'
    BOLD_YELLOW = '\033[1;33m'
    BOLD_BLUE = '\033[1;34m'
    BOLD_PURPLE = '\033[1;35m'
    BOLD_CYAN = '\033[1;36m'
    BOLD_WHITE = '\033[1;37m'
    test1 = '\033[1;38'

# Function to print the color name and sample message
def print_color_message(color_code, color_name):
    print(f"{color_code}This is a {color_name} message{AnsiColorCodes.END}")

# Print messages in all defined colors
def main():
    # print_color_message(AnsiColorCodes.BLACK, "black")
    # print_color_message(AnsiColorCodes.RED, "red")
    # print_color_message(AnsiColorCodes.GREEN, "green")
    # print_color_message(AnsiColorCodes.YELLOW, "yellow")
    # print_color_message(AnsiColorCodes.BLUE, "blue")
    # print_color_message(AnsiColorCodes.PURPLE, "purple")
    # print_color_message(AnsiColorCodes.CYAN, "cyan")
    # print_color_message(AnsiColorCodes.WHITE, "white")
    # print_color_message(AnsiColorCodes.BOLD_BLACK, "bold black")
    # print_color_message(AnsiColorCodes.BOLD_RED, "bold red")
    # print_color_message(AnsiColorCodes.BOLD_GREEN, "bold green")
    # print_color_message(AnsiColorCodes.BOLD_YELLOW, "bold yellow")
    # print_color_message(AnsiColorCodes.BOLD_BLUE, "bold blue")
    # print_color_message(AnsiColorCodes.BOLD_PURPLE, "bold purple")
    # print_color_message(AnsiColorCodes.BOLD_CYAN, "bold cyan")
    # print_color_message(AnsiColorCodes.BOLD_WHITE, "bold white")
    for i in range(30, 100):
      print_color_message(f'\033[1;{i}m', i)

if __name__ == "__main__":
    main()
