#!/bin/bash

set -e

# Script to build and install habitat-sim and habitat-lab
# Usage: bash scripts/install_habitat.sh

# Navigate to habitat-sim directory
cd 3rdparty/habitat-sim

echo "[1/3] Installing habitat-sim (editable mode)"
CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
pip install . -v
CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
pip install -e .

# Navigate to habitat-lab directory
cd ../habitat-lab

echo "[2/3] Installing habitat-lab (editable mode)"
pip install -e habitat-lab

# Navigate back to project root
cd ../..

echo "[3/3] Finished installing habitat-sim and habitat-lab"