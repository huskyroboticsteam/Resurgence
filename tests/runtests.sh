#!/bin/bash

# assumes that source code files are already compiled
# uses the .o files in the build directory

for ARG in "$@"; do
    echo "running test $ARG"
    g++ $ARG -L ../build/*.a -I ../src/
    ./a.out
    rm a.out
done
