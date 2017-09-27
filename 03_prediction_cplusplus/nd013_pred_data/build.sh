#!/bin/bash

DIRECTORY="./Debug"

function build_debug() {
    echo "Building the debug directory"
    mkdir $DIRECTORY
    cd $DIRECTORY
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    make
    cd ..
}

if [ -d "$DIRECTORY" ];then
    echo "Directory exist, it will be removed"
    rm -rf $DIRECTORY
    build_debug
fi

echo "Creating Debug directory"
build_debug

