import matplotlib.pyplot as plt
import numpy as np
import argparse

# Function to read data from the file
def read_data(filename):
    data = {}
    with open(filename, 'r') as file:
        current_weight = None
        for line in file:
            line = line.strip()
            if line.startswith("Averages for"):
                current_weight = line.split("=")[-1].strip(":")
                data[current_weight] = {}
            elif ":" in line and current_weight:
                key, value = line.split(":")
                data[current_weight][key.strip()] = float(value.strip())
    return data

def sort_metrics_by_weight(data):
    """Sorts metrics for each weight by their runtime in descending order."""
    sorted_metrics = {}
    for weight, metrics in data.items():
        # Sort metrics by value (descending order)
        sorted_metrics[weight] = sorted(metrics.items(), key=lambda x: x[1], reverse=True)
    return sorted_metrics

def main():
    # Argument parser
    parser = argparse.ArgumentParser(description="Generate horizontal overlapping bar plots from data in a file.")
    parser.add_argument("filename", type=str, help="Path to the input file containing the data")
    parser.add_argument("--output", type=str, default="sorted_fully_overlapping_plot.png", help="Output file name for the plot (default: sorted_fully_overlapping_plot.png)")
    args = parser.parse_args()

    # Read data from the file
    data = read_data(args.filename)

    # Sort metrics for each weight by runtime
    sorted_metrics = sort_metrics_by_weight(data)

    # Prepare data for plotting
    weights = list(data.keys())
    metrics_by_weight = {weight: [metric for metric, _ in sorted_metrics[weight]] for weight in weights}
    values_by_weight = {weight: [value for _, value in sorted_metrics[weight]] for weight in weights}

    # Define consistent colors for each metric
    all_metrics = sorted(list(next(iter(data.values())).keys()))  # All metric names
    color_mapping = {metric: plt.cm.tab10(i) for i, metric in enumerate(all_metrics)}

    # Horizontal overlapping bar plot
    plt.figure(figsize=(10, 6))
    y = np.arange(len(weights))  # Positions for the weights on the y-axis
    bar_height = 0.4  # Single height since bars fully overlap

    for weight_idx, weight in enumerate(weights):
        for metric_idx, metric in enumerate(metrics_by_weight[weight]):
            # Plot each metric for the current weight
            plt.barh(
                y[weight_idx], 
                values_by_weight[weight][metric_idx], 
                height=bar_height, 
                color=color_mapping[metric],  # Use consistent color
                label=metric if weight_idx == 0 else None,  # Add label only once
                alpha=0.7
            )

    # Adding labels and title
    plt.ylabel('Weights (w)', fontsize=12)
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.title('Comparison of Runtime Metrics Sorted by Slower Time', fontsize=14)
    plt.yticks(y, weights)
    plt.legend(loc='upper right', fontsize=10)
    plt.grid(axis='x', linestyle='--', alpha=1)

    # Save and show plot
    plt.tight_layout()
    plt.savefig(args.output)
    print(f"Plot saved as {args.output}")
    plt.show()

if __name__ == "__main__":
    main()
