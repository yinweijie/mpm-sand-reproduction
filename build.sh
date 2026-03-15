#!/bin/bash

set -e

mode=${1:-release}
scene=${2:-scenes/hourglass.json}
target=klar2016_sand

if [ "$mode" == "release" ]; then
    echo "Building in release mode..."
    conan install . --build=missing -s="compiler.cppstd=20"
    cmake --list-presets
    cmake --preset=conan-release
    cmake --build --preset=conan-release
    ./build/Release/${target} --scene "${scene}"
    rsync -avh ./build/Release/compile_commands.json ./build/
else
    echo "Building in debug mode..."
    conan install . --build=missing -s="build_type=Debug" -s="compiler.cppstd=20"
    cmake --preset=conan-debug -DCMAKE_BUILD_TYPE=Debug
    cmake --build --preset=conan-debug
    ./build/Debug/${target} --scene "${scene}"
    rsync -avh ./build/Debug/compile_commands.json ./build/
fi
