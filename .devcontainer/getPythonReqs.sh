#!/bin/bash


# Create a virtual environment
python3 -m venv /workspaces/ITA-CBS_dev-main/python/myenv

# Activate the virtual environment
source /workspaces/ITA-CBS_dev-main/python/myenv/bin/activate

# Install the required packages
pip install -r /workspaces/ITA-CBS_dev-main/.devcontainer/requirements.txt
