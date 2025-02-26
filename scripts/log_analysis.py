import re
import sys

def analyze_log(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    error_pattern = re.compile(r'error', re.IGNORECASE)
    warning_pattern = re.compile(r'warning', re.IGNORECASE)

    errors = []
    warnings = []

    for line in lines:
        if error_pattern.search(line):
            errors.append(line.strip())
        elif warning_pattern.search(line):
            warnings.append(line.strip())

    print("Errors:")
    for error in errors:
        print(error)

    print("\nWarnings:")
    for warning in warnings:
        print(warning)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python log_analysis.py <log_file_path>")
        sys.exit(1)

    log_file_path = sys.argv[1]
    analyze_log(log_file_path)
