import os
import yaml
import argparse
import re
from collections import defaultdict

# Define the metrics to track
METRICS = [
    'newnode_runtime',
    'focal_score_time',
    'firstconflict_runtime',
    'runtime',
    'lowlevel_search_time',
]

def extract_metrics_from_yaml(file_path, metrics, lines_to_read=20):
    print(f"Processing {file_path}")
    try:
        with open(file_path, 'r') as file:
            head_data = ''.join(file.readline() for _ in range(lines_to_read))
            data = yaml.safe_load(head_data)
            if data and 'statistics' in data:
                return {metric: float(data['statistics'].get(metric, 0)) for metric in metrics}
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
    return {}


def get_w_from_folder_name(folder_name):
    """
    Extract the value of 'w' from the folder name using regex.
    """
    match = re.search(r'_w=([0-9.]+)', folder_name.strip())
    return float(match.group(1)) if match else None

def process_directory(base_dir):
    """
    Traverse the directory structure, extract metrics, and calculate averages.
    """
    w_metrics = defaultdict(lambda: defaultdict(lambda: {'sum': 0, 'count': 0}))

    # Traverse the directory tree and process files
    for root, dirs, files in os.walk(base_dir):
        w_value = get_w_from_folder_name(os.path.basename(root))
        if w_value is not None:
            for sub_dir in dirs:
                sub_dir_path = os.path.join(root, sub_dir)
                yaml_files = [
                    os.path.join(sub_dir_path, file_name)
                    for file_name in os.listdir(sub_dir_path)
                    if file_name.endswith(".yaml")
                ]

                # Process each YAML file
                for file_path in yaml_files:
                    metrics = extract_metrics_from_yaml(file_path, METRICS)
                    for metric, value in metrics.items():
                        w_metrics[w_value][metric]['sum'] += value
                        w_metrics[w_value][metric]['count'] += 1

    # Calculate averages for each 'w'
    averages = {
        w_value: {
            metric: (data['sum'] / data['count']) if data['count'] > 0 else 0
            for metric, data in metrics.items()
        }
        for w_value, metrics in w_metrics.items()
    }

    return averages

def save_averages_to_file(averages, output_file):
    """
    Save the averages to a file.
    """
    try:
        with open(output_file, 'w') as file:
            for w_value, metrics in sorted(averages.items()):
                file.write(f"Averages for w={w_value}:\n")
                for metric, avg_value in metrics.items():
                    file.write(f"  {metric}: {avg_value}\n")
    except Exception as e:
        print(f"Error writing to file {output_file}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a directory of YAML files and calculate metrics averages.")
    parser.add_argument("directory", type=str, help="The base directory to process")
    parser.add_argument("--output", type=str, default="averages.txt", help="Output file to save averages")
    args = parser.parse_args()

    base_directory = args.directory
    output_file = args.output

    if not os.path.isdir(base_directory):
        print(f"Error: The provided path '{base_directory}' is not a valid directory.")
        exit(1)

    averages = process_directory(base_directory)
    save_averages_to_file(averages, output_file)

    print(f"Averages have been saved to {output_file}.")
