import os
import subprocess
import sys

# Metrics we care about
METRICS = [
    "newnode_runtime",
    "focal_score_time",
    "firstconflict_runtime",
    "runtime",
    "lowlevel_search_time",
]

def parse_statistics(output_file):
    """
    Parses the statistics section of the output file into a dictionary.
    Only extracts metrics from the defined METRICS list.
    """
    statistics = {metric: 0.0 for metric in METRICS}
    counts = {metric: 0 for metric in METRICS}

    in_statistics = False

    try:
        with open(output_file, 'r') as f:
            for line in f:
                # Check if we are in the "statistics" section
                if line.strip() == "statistics:":
                    in_statistics = True
                    continue

                if in_statistics:
                    # Stop reading if we encounter a non-indented line
                    if not line.startswith("  "):
                        break

                    # Extract metric and value
                    parts = line.strip().split(":")
                    if len(parts) == 2:
                        key, value = parts[0].strip(), parts[1].strip()
                        if key in METRICS:
                            statistics[key] += float(value)
                            counts[key] += 1

    except FileNotFoundError:
        print(f"Error: {output_file} not found.")

    return statistics, counts


def calculate_averages(total_statistics, total_counts):
    """
    Calculates averages for the metrics from the aggregated totals and counts.
    """
    averages = {}
    for metric in METRICS:
        averages[metric] = (
            total_statistics[metric] / total_counts[metric]
            if total_counts[metric] > 0
            else 0.0
        )
    return averages


def process_files(input_dir):
    output_file = 'sample.yaml'

    # Check if the directory exists
    if not os.path.isdir(input_dir):
        print(f"Error: {input_dir} is not a valid directory.")
        return

    # Initialize totals and counts
    total_statistics = {metric: 0.0 for metric in METRICS}
    total_counts = {metric: 0 for metric in METRICS}

    # Iterate through all files in the directory
    for filename in os.listdir(input_dir):
        filepath = os.path.join(input_dir, filename)

        if os.path.isfile(filepath):  # Ensure it's a file
            print(f"Processing file: {filepath}")

            # Construct the command with the input file
            command = f'./ECBS_parallel -i {filepath} -o {output_file}'

            # Execute the command
            try:
                subprocess.run(command, shell=True, check=True)
                print(f"Command executed successfully for {filepath}")
            except subprocess.CalledProcessError as e:
                print(f"Command failed for {filepath}: {e}")
                continue

            # Parse the statistics section of the output file
            statistics, counts = parse_statistics(output_file)

            # Aggregate the totals and counts
            for metric in METRICS:
                total_statistics[metric] += statistics[metric]
                total_counts[metric] += counts[metric]

    # Delete the output file at the end
    try:
        if os.path.exists(output_file):
            os.remove(output_file)
            print(f"Deleted {output_file}")
    except Exception as e:
        print(f"Failed to delete {output_file}: {e}")

    # Calculate and display averages
    averages = calculate_averages(total_statistics, total_counts)
    print("\nAverages:")
    for key, value in averages.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    # Check if an argument is provided
    if len(sys.argv) < 2:
        print("Usage: python script.py <input_directory>")
        sys.exit(1)

    # Get the input directory from command-line arguments
    input_directory = sys.argv[1]
    process_files(input_directory)
