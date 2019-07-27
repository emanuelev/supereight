#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# Required dependencies
sudo apt -y install build-essential git cmake libeigen3-dev libgtest-dev

# Optional dependencies
sudo apt -y install freeglut3-dev libopenni2-dev libpapi-dev qtbase5-dev

# Source dependencies
sophus_dir="$HOME/Documents/Source/Sophus"
mkdir -p "$sophus_dir/build"
git clone --depth=1 https://github.com/strasdat/Sophus.git "$sophus_dir"
cd "$sophus_dir/build" && cmake -DCMAKE_BUILD_TYPE=Release ..
cd "$sophus_dir/build" && make && sudo make install

