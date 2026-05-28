#!/bin/bash
# CIWS launcher - activates venv and runs the main program

# Activate the CVsys virtual environment
source /home/CIWS/.virtualenvs/CVsys/bin/activate

# cd into the project directory so relative paths work
cd /home/CIWS

# Run the program, passing through any arguments
python3 CIWS_Combined.py "$@"