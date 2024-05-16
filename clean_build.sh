#!/bin/bash

build_directory="build"

# Check if the build directory exists
if [ -d "$build_directory" ]; then
    echo "deleting existing build directory..."
    rm -rf build
fi
echo "Creating and entering build directory..."
mkdir "$build_directory" && cd "$build_directory"

echo "runnig cmake .."
cmake ..
echo "runnig make .."
make
#improves:
#
#using mouse to target plates
#create save and load functionality with jason for example.
# move canon with arrows
# move cannon with mose
# apply rts game style to control the cannon or add jetpack to the cannon
# 