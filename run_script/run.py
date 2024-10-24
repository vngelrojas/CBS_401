#!/usr/bin/env python3

'''
# run.py - A script to manage CMake build commands and execute built binaries with categorized outputs.
# NOTE: Activate environment first, look at readme.md
#       run: python run.py configure 
#       now you can do: python run.py <COMMAND> -i ../map_file/<FILE FOLDER>/ -c
#       -c is to run all of the files in the directory given
#
#       Note: And when pushing only put the run.py file no need for the other stuff in run_script folder
'''

import argparse
import os
import sys
import subprocess
import shutil
from pathlib import Path
from typing import Tuple, List

DEFAULT_INPUT_PATH = Path("../map_file/debug_cbs_data.yaml") # temp holder
BASE_OUTPUT_DIR = Path("../outputs")
DEFAULT_TIMEOUT_DURATION = 5  # seconds

SUCCESS_FILES: List[str] = []
FAILED_FILES: List[str] = []
SKIPPED_FILES: List[str] = []

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
    return parser.parse_args()

def ensure_input_path(input_path: Path):
    if not input_path.exists():
        print(f"Error: Input path '{input_path}' doesn't exist.")
        sys.exit(1)

def derive_output_path(cmd: str, input_file: Path) -> Path:
    base_input_dir = Path("../map_file")
    if base_input_dir in input_file.parents:
        relative_path = input_file.relative_to(base_input_dir)
    else:
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

def build_and_execute_single(target_name: str, executable: str, file_path: Path, timeout_duration: int) -> bool:
    derived_output = derive_output_path(target_name, file_path)
    ensure_output_directory(derived_output)

    print(f"Building target: {target_name} for file: {file_path}")
    build_result = subprocess.run(["cmake", "--build", ".", "--target", target_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if build_result.returncode != 0:
        print(f"Error: {target_name} build failed for '{file_path}'.")
        FAILED_FILES.append(f"{file_path} (Build Failed)")
        return False

    print(f"Executing: ./{executable} -i \"{file_path}\" -o \"{derived_output}\" with a timeout of {timeout_duration} sec.")
    try:
        subprocess.run(
            [f"./{executable}", "-i", str(file_path), "-o", str(derived_output)],
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
        print(f"'{file_path}' failed. Look at execution.log for details.")
        FAILED_FILES.append(f"{file_path} (Execution Failed)")
        return False

def build_and_execute_batch(target_name: str, executable: str, directory_path: Path, timeout_duration: int, continue_on_failure: bool):
    yaml_files = list(directory_path.glob("*.yaml"))
    
    if not yaml_files:
        print(f"No yaml tests found in directory '{directory_path}'.")
        sys.exit(1)

    for file in yaml_files:
        success = build_and_execute_single(target_name, executable, file, timeout_duration)
        if not success:
            print(f"Error encountered with file '{file}'.")
            if continue_on_failure:
                print("Continuing to next file due to '-c' flag.")
                continue
            else:
                print("Stopping further processing due to failure.")
                break

def handle_commands(target_name: str, executable: str, input_path: Path, timeout_duration: int, continue_on_failure: bool):
    if input_path.is_dir():
        print(f"Input path '{input_path}' is a directory. Processing all .yaml files inside.")
        build_and_execute_batch(target_name, executable, input_path, timeout_duration, continue_on_failure)
    elif input_path.is_file():
        print(f"Input path '{input_path}' is a file. Processing it.")
        build_and_execute_single(target_name, executable, input_path, timeout_duration)
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

def main():
    args = usage()

    command = args.command
    input_path = Path(args.input)
    continue_on_failure = args.cont
    timeout_duration = args.timeout
 
    ensure_input_path(input_path)

    if command == "configure":
        print("Running: cmake ..")
        try:
            with open("build_config.log", "w") as log_file:
                result = subprocess.run(["cmake", ".."], stdout=log_file, stderr=subprocess.STDOUT)
            if result.returncode != 0:
                print("Error: Configuration failed.")
                sys.exit(1)
        except Exception as e:
            print(f"Error during configuration: {e}")
            sys.exit(1)
    else:
        target, executable = map_command_to_target(command) # target - build target in cmake, executable - output binary
        if not target or not executable:
            print(f"Error: Invalid command '{command}'.")
            usage()
        
        handle_commands(target, executable, input_path, timeout_duration, continue_on_failure)

    print("\nSummary:")
    print(f"Successful: {len(SUCCESS_FILES)}")
    for f in SUCCESS_FILES:
        print(f"  - {f}")
    
    print(f"\nFailed: {len(FAILED_FILES)}")
    for f in FAILED_FILES:
        print(f"  - {f}")
    
    print(f"\nSkipped: {len(SKIPPED_FILES)}")
    for f in SKIPPED_FILES:
        print(f"  - {f}")

    if FAILED_FILES:
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()
