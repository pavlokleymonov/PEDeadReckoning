#!/bin/bash

CMAKE_FLAGS=$@

mkdir -p ./out

cd ./out

echo "Build started at: $(tput setaf 3)$(date +%X)$(tput sgr0)"

CORES=$(nproc)
echo "CPU count for build:$CORES"

cmake $CMAKE_FLAGS ..

if [ $? -ne 0 ]; then
	echo "Errors generating makefiles"
	exit 1
fi

make -j$CORES
if [ $? -ne 0 ]; then
	echo "Errors building unittest"
	exit 1
fi

make test
if [ $? -ne 0 ]; then
	echo "Errors running test"
	exit 1
fi

make coverage
if [ $? -ne 0 ]; then
	echo "Errors creating coverage report"
	exit 1
fi

#TIME_END=$(date +%X)
echo "Build finished at: $(tput setaf 3)$(date +%X)$(tput sgr0)"

browse ./coverage.html
if [ $? -ne 0 ]; then
	echo "Errors openning coverage report"
	exit 1
fi
