import os
import yaml
import pandas as pd
import matplotlib.pyplot as plt

# Path to the folder where YAML files are located
yaml_folder = '../outputs/random-32-32-10_015/ECBS_parallel'

# Function to extract relevant data from a single YAML file
def extract_metrics_from_yaml(file_path):
    print(f"Processing file: {file_path}")
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    
    # Extract relevant statistics (runtime, cost, etc.)
    statistics = yaml_data.get('statistics', {})
    runtime = statistics.get('runtime', None)
    cost = statistics.get('cost', None)

    # Ignore files where the cost is greater than 1500
    if cost is not None and cost > 1500:
        print(f"Ignoring file {file_path} due to cost > 1500.")
        return None

    # Extract number of agents from the file name (e.g., maze-32-32-2_agents_10_test_X.yaml)
    if '_agents_' in file_path:
        try:
            num_agents = file_path.split('_agents_')[1].split('_')[0]
        except IndexError:
            print(f"Error extracting num_agents from file name {file_path}. Skipping file.")
            return None
    else:
        print(f"Warning: Could not extract num_agents from file name {file_path}. Skipping file.")
        return None

    print(f"Extracted data: runtime={runtime}, cost={cost}, num_agents={num_agents}")

    return {
        'file': os.path.basename(file_path),
        'num_agents': int(num_agents),
        'runtime': runtime,
        'cost': cost
    }

# Collect all YAML files in the folder and skip non-YAML files
yaml_files = [os.path.join(yaml_folder, f) for f in os.listdir(yaml_folder) if f.endswith('.yaml')]

# Print the YAML files found
print("YAML files found:", yaml_files)

# Extract data from all YAML files
extracted_data = [extract_metrics_from_yaml(file) for file in yaml_files]
# Filter out any None values in case some files were skipped
extracted_data = [data for data in extracted_data if data is not None]

# Convert to a DataFrame for easier analysis
df = pd.DataFrame(extracted_data)

# Verify if data was extracted
print("Extracted DataFrame:")
print(df)

# Check if DataFrame is empty
if df.empty:
    print("No valid data extracted. Exiting script.")
    exit()

# Save the extracted data to a CSV file (optional)
df.to_csv('extracted_data.csv', index=False)

# Visualize runtime vs number of agents
plt.figure(figsize=(8, 6))
plt.plot(df['num_agents'], df['runtime'], marker='o', label='Runtime')
plt.xlabel('Number of Agents')
plt.ylabel('Runtime (seconds)')
plt.title('Runtime vs Number of Agents')
plt.legend()
plt.grid(True)
plt.savefig('runtime_vs_agents_ECBS_parallel.png')  # Save plot as PNG file
plt.show()

# Visualize cost (SOC optimality) vs number of agents
plt.figure(figsize=(8, 6))
plt.plot(df['num_agents'], df['cost'], marker='o', color='r', label='SOC Optimality (Cost)')
plt.xlabel('Number of Agents')
plt.ylabel('Cost (SOC Optimality)')
plt.title('Cost (SOC Optimality) vs Number of Agents')
plt.legend()
plt.grid(True)
plt.savefig('cost_vs_agents_ECBS_parallel.png')  # Save plot as PNG file
plt.show()

print("Plots and CSV generated successfully.")
