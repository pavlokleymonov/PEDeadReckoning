#!/bin/bash

CMAKE_FLAGS=$@

mkdir -p ./out/ut

cd ./out/ut

CORES=$(nproc)
echo "CPU count for build:$CORES"

cmake -DENABLE_COVERAGE=1 -DENABLE_TESTS=1 $CMAKE_FLAGS ../../test/ut

if [ $? -ne 0 ]; then
	echo "Errors generating makefiles"
	exit 1
fi

echo "Build started at: $(tput setaf 3)$(date +%X)$(tput sgr0)"

make -j$CORES
if [ $? -ne 0 ]; then
	echo "Errors building unittest"
	exit 1
fi

#TIME_END=$(date +%X)
echo "Build finished at: $(tput setaf 3)$(date +%X)$(tput sgr0)"

make coverage
if [ $? -ne 0 ]; then
	echo "Errors creating coverage report"
	exit 1
fi

browse ./coverage.html
if [ $? -ne 0 ]; then
	echo "Errors openning coverage report"
	exit 1
fi
