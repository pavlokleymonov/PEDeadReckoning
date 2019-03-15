#!/bin/bash

CMAKE_FLAGS=$@

mkdir -p ./test/ut/b

cd ./test/ut/b

cmake -DENABLE_COVERAGE=1 -DENABLE_TESTS=1 $CMAKE_FLAGS ..

if [ $? -ne 0 ]; then
	echo "Errors generating makefiles"
	exit 1
fi

echo "Build started at"
date +%X
make
if [ $? -ne 0 ]; then
	echo "Errors building unittest"
	exit 1
fi
echo "Build finished at"
date +%X

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
