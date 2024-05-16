#!/bin/bash

build_directory="build"

# Check if the build directory exists
if [ -d "$build_directory" ]; then
    echo "Entering existing build directory..."
    cd "$build_directory"
else
    echo "Build directory not found. Creating and entering..."
    mkdir "$build_directory" && cd "$build_directory"
fi

cmake ..
make

#improves:
#
#using mouse to target plates
#create save and load functionality with jason for example.
# move canon with arrows
# move cannon with mose
# apply rts game style to control the cannon or add jetpack to the cannon
# 