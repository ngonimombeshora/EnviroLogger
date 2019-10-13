#!/bin/bash

case "$1" in
raspberry)
    cd WiringPi
    ./build
    gpio -v
    cd ..
    make clean all target=raspberry
    exit 0
    ;;
linux)
    make clean all
    exit 0
    ;;
esac

echo "Please specify platform: raspberry, linux"
exit 1
