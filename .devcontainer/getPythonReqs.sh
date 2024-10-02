#!/bin/bash


# Create a virtual environment
python3 -m venv /workspaces/CBS_401/python/myenv

# Activate the virtual environment
source /workspaces/CBS_401/python/myenv/bin/activate

# Install the required packages
pip install -r /workspaces/CBS_401/.devcontainer/requirements.txt
