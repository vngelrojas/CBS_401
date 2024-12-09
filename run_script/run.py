#!/usr/bin/env python3

'''
# run.py - A script to manage CMake build commands and execute built binaries with categorized outputs.
#          A summary page will me output into 'summary.txt'
# NOTE: - Activate environment first.
#               - if not activated try `source /workspaces/CBS_401/python/myenv/bin/activate` 
#       - cd into run_script folder
#       - run: `python run.py configure`
#       - now you can do:
#               - `python run.py <COMMAND> -i ../map_file/<FILE FOLDER NAME>/ -c -w <NUMBER>`
#       - Extra Info:
#           -c is to run all of the files in the directory given
#           -w is optional and is only for ECBS command. Number can be anywhere from 1.0 to 1.2
#           -t can be used to set the timeout duration in seconds
#
#       Example: python run.py ECBS -i ../map_file/Boston_0_256_020/ -c -w 1.02
'''

import argparse
import os
import sys
import subprocess
from pathlib import Path
from typing import Tuple, List, Optional

DEFAULT_INPUT_PATH = Path("../map_file/debug_cbs_data.yaml") # temp holder
BASE_OUTPUT_DIR = Path("../outputs")
DEFAULT_TIMEOUT_DURATION = 13  # seconds

# Global Lists to Track File Statuses
SUCCESS_FILES: List[str] = []
FAILED_FILES: List[str] = []
SKIPPED_FILES: List[str] = []

# Define ECBS-related commands (Only 'ECBS' for -w option. Add more later)
ECBS_COMMANDS = {"ECBS","ECBS-p"}

def usage():
    parser = argparse.ArgumentParser(
        description="Manage CMake build commands and execute built binaries with categorized outputs.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "command",
        choices=["configure", "CBS", "CBS-p", "ECBS", "ECBS-p", "CBS-d", "ECBS-d"],
        help=(
            "Commands:\n"
            "  configure      Run CMake to configure the build system.\n"
            "  CBS            Build the CBS target and execute it.\n"
            "  CBS-p          Build the CBS_parallel target and execute it.\n"
            "  ECBS           Build the ECBS target and execute it.\n"
            "  ECBS-p         Build the ECBS_parallel target and execute it.\n"
            "  CBS-d          Build the CBS_distributed target and execute it.\n"
            "  ECBS-d         Build the ECBS_distributed target and execute it."
        )
    )
    parser.add_argument(
        "-i", "--input",
        type=str,
        default=str(DEFAULT_INPUT_PATH),
        help=f"Specify input yaml file or directory (default is {DEFAULT_INPUT_PATH})."
    )
    parser.add_argument(
        "-c", "--cont",
        action="store_true",
        help="Continue processing the rest of the files even if a failure or timeout occurs."
    )
    parser.add_argument(
        "-t", "--timeout",
        type=int,
        default=DEFAULT_TIMEOUT_DURATION,
        help=f"Set the timeout duration in seconds (default is {DEFAULT_TIMEOUT_DURATION} seconds)."
    )
    parser.add_argument(
        "-w", "--weight",
        type=float,
        default=None,
        help="Optional weight parameter for ECBS command (e.g., -w 1.02)."
    )
    return parser.parse_args()

def ensure_input_path(input_path: Path):
    if not input_path.exists():
        print(f"Error: Input path '{input_path}' doesn't exist.")
        sys.exit(1)

def derive_output_path(cmd: str, input_file: Path) -> Path:
    base_input_dir = Path("../map_file")
    try:
        relative_path = input_file.relative_to(base_input_dir)
    except ValueError:
        # If input_file is not under base_input_dir
        relative_path = input_file.name

    if input_file.is_file():
        relative_dir = relative_path.parent
    elif input_file.is_dir():
        relative_dir = relative_path
    else:
        relative_dir = Path("")

    output_path = BASE_OUTPUT_DIR / relative_dir / cmd / input_file.name
    return output_path

def ensure_output_directory(output_file: Path):
    output_dir = output_file.parent
    if not output_dir.exists():
        print(f"Output directory '{output_dir}' doesn't exist. Will create.")
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            print(f"Error: Failed to create output directory '{output_dir}': {e}")
            sys.exit(1)

def build_and_execute_single(target_name: str, executable: str, file_path: Path, timeout_duration: int, weight: Optional[float] = None) -> bool:
    derived_output = derive_output_path(target_name, file_path)
    ensure_output_directory(derived_output)

    print(f"Building target: {target_name} for file: {file_path}")
    build_result = subprocess.run(
        ["cmake", "--build", ".", "--target", target_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    if build_result.returncode != 0:
        print(f"Error: {target_name} build failed for '{file_path}'.")
        FAILED_FILES.append(f"{file_path} (Build Failed)")
        return False

    # Construct the execution command
    exec_command = [f"./{executable}", "-i", str(file_path), "-o", str(derived_output)]
    
    # Append -w <NUMBER> for applicable commands
    if weight is not None:
        exec_command.extend(["-w", str(weight)])

    print(f"Executing: {' '.join(exec_command)} with a timeout of {timeout_duration} sec.")
    try:
        subprocess.run(
            exec_command,
            timeout=timeout_duration,
            check=True
        )
        print(f"Successfully processed file: {file_path}")
        SUCCESS_FILES.append(str(file_path))
        return True
    except subprocess.TimeoutExpired:
        print(f"'{file_path}' took too long. Skipping.")
        SKIPPED_FILES.append(f"{file_path} (Timeout)")
        return True
    except subprocess.CalledProcessError:
        print(f"'{file_path}' failed.")
        FAILED_FILES.append(f"{file_path} (Execution Failed)")
        return False

def build_and_execute_batch(target_name: str, executable: str, directory_path: Path, timeout_duration: int, continue_on_failure: bool, weight: Optional[float] = None):
    yaml_files = list(directory_path.glob("*.yaml"))
    
    if not yaml_files:
        print(f"No yaml tests found in directory '{directory_path}'.")
        sys.exit(1)

    for file in yaml_files:
        success = build_and_execute_single(target_name, executable, file, timeout_duration, weight)
        if not success:
            print(f"Error encountered with file '{file}'.")
            if continue_on_failure:
                print("Continuing to next file due to '-c' flag.")
                continue
            else:
                print("Stopping further processing due to failure.")
                break

def handle_commands(target_name: str, executable: str, input_path: Path, timeout_duration: int, continue_on_failure: bool, weight: Optional[float] = None):
    if input_path.is_dir():
        print(f"Input path '{input_path}' is a directory. Processing all .yaml files inside.")
        build_and_execute_batch(target_name, executable, input_path, timeout_duration, continue_on_failure, weight)
    elif input_path.is_file():
        print(f"Input path '{input_path}' is a file. Processing it.")
        build_and_execute_single(target_name, executable, input_path, timeout_duration, weight)
    else:
        print(f"Error: '{input_path}' isn't a file or a directory.")
        sys.exit(1)

def map_command_to_target(cmd: str) -> Tuple[str, str]:
    mapping = {
        "CBS": ("CBS", "CBS"),
        "CBS-p": ("CBS_parallel", "CBS_parallel"),
        "ECBS": ("ECBS", "ECBS"),
        "ECBS-p": ("ECBS_parallel", "ECBS_parallel"),
        "CBS-d": ("CBS_distributed", "CBS_distributed"),
        "ECBS-d": ("ECBS_distributed", "ECBS_distributed"),
    }
    return mapping.get(cmd, ("", ""))
    
def write_summary(target_name: str, input_path: Path, timeout_duration: int):
    # Derive the summary file path based on the input directory or file
    if input_path.is_dir():
        summary_file_path = BASE_OUTPUT_DIR / input_path.name / "summary.txt"
    else:
        summary_file_path = derive_output_path(target_name, input_path).parent / "summary.txt"

    # Ensure the directory for the summary file exists
    summary_file_path.parent.mkdir(parents=True, exist_ok=True)

    # Write summary to the summary file
    with open(summary_file_path, "w") as summary_file:
        # Write command and timeout details at the top
        summary_file.write("Execution Summary:\n")
        summary_file.write(f"Command: {target_name}\n")
        summary_file.write(f"Timeout Duration: {timeout_duration} seconds\n\n")

        SUCCESS_FILES.sort()
        FAILED_FILES.sort()
        SKIPPED_FILES.sort()

        summary_file.write("Summary:\n")
        summary_file.write(f"Successful: {len(SUCCESS_FILES)}\n")
        for f in SUCCESS_FILES:
            summary_file.write(f"  - {f}\n")

        summary_file.write(f"\nFailed: {len(FAILED_FILES)}\n")
        for f in FAILED_FILES:
            summary_file.write(f"  - {f}\n")

        summary_file.write(f"\nSkipped: {len(SKIPPED_FILES)}\n")
        for f in SKIPPED_FILES:
            summary_file.write(f"  - {f}\n")

    print(f"Summary written to: {summary_file_path}")


def main():
    args = usage()

    command = args.command
    input_path = Path(args.input)
    continue_on_failure = args.cont
    timeout_duration = args.timeout
    weight = args.weight
    ensure_input_path(input_path)

    # Validate -w usage: only for ECBS
    if weight is not None and command not in ECBS_COMMANDS:
        print(f" Command '{command}' does not support '-w'.")
        sys.exit(1)

    if command == "configure":
        print("Running: cmake ..")
        try:
            with open("build_config.log", "w") as log_file:
                result = subprocess.run(
                    ["cmake", ".."],
                    stdout=log_file,
                    stderr=subprocess.STDOUT
                )
            if result.returncode != 0:
                print("Error: Configuration failed.")
                sys.exit(1)
        except Exception as e:
            print(f"Error during configuration: {e}")
            sys.exit(1)
    else:
        target, executable = map_command_to_target(command)
        if not target or not executable:
            print(f"Error: Invalid command '{command}'.")
            usage()

        handle_commands(target, executable, input_path, timeout_duration, continue_on_failure, weight)

    # Write the summary to the derived directory
    write_summary(command, input_path, timeout_duration)

    if FAILED_FILES:
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()
